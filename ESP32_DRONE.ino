/*
  ESP32_DRONE - Controlul unei drone cu patru motoare, în configurație X
  ------------------------------------------------------------------

  Fluxul principal:
  1. Citește accelerația și vitezele unghiulare de la MPU6050 prin I2C.
  2. Estimează unghiurile Pitch și Roll cu un filtru complementar.
  3. Preia comenzile telecomenzii prin ESP-NOW.
  4. Calculează corecțiile pentru Pitch/Roll (unghi) și Yaw (viteză).
  5. Combină throttle-ul și corecțiile în patru comenzi pentru ESC-uri.

  Convenții și unități:
  - Pitch: înclinarea înainte/înapoi; Roll: înclinarea laterală.
  - Yaw: rotația în jurul axei verticale. Se controlează viteza de
    rotație, fără estimarea sau menținerea unui unghi Yaw absolut.
  - Asocierea gx -> Pitch și gy -> Roll este cea folosită în acest cod;
    interpretarea fizică depinde de orientarea senzorului pe cadru.
  - Unghiuri: grade; viteze unghiulare: grade/s; intervale dt: secunde.
  - Valorile trimise ESC-urilor reprezintă durate de impuls în
    microsecunde (us), nu turații măsurate ale motoarelor.
  - Pozițiile motoarelor și sensurile CW/CCW sunt descrise privind
    drona de sus: CW = orar, CCW = antiorar.
    M1: față-stânga CCW; M2: față-dreapta CW;
    M3: spate-stânga CW; M4: spate-dreapta CCW.

  Condiții de activare a ieșirilor:
  - Armarea este memorată la o tranziție observată a comutatorului
    din oprit în pornit, dacă throttle-ul este cel mult centru + marjă.
  - Ieșirile sunt aduse la PWM_MIN când drona nu este armată, când
    comunicația expiră sau când eroarea IMU a fost memorată.
  - Failsafe-ul de comunicație nu șterge starea de armare memorată;
    comenzile pot redeveni active când comunicația revine.
  - Monitorizarea bateriei este opțională și dezactivată implicit.

  Pinii, calibrările, pragurile și coeficienții PID sunt parametrii
  acestei configurații hardware, nu valori universale pentru orice dronă.
*/

#include <Wire.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// ---------- Ieșiri de comandă pentru ESC-uri ----------
const int PIN_ESC_M1 = 13; // M1: față-stânga, CCW (vedere de sus).
const int PIN_ESC_M2 = 14; // M2: față-dreapta, CW (vedere de sus).
const int PIN_ESC_M3 = 27; // M3: spate-stânga, CW (vedere de sus).
const int PIN_ESC_M4 = 26; // M4: spate-dreapta, CCW (vedere de sus).

Servo escM1, escM2, escM3, escM4;

// ---------- LED de stare ----------
const int PIN_LED = 5;
const unsigned long LED_BLINK_INTERVAL_MS = 500; // Schimbă starea la 500 ms; un ciclu aprins/stins durează aproximativ 1 s.

const int PWM_MIN = 1000;   // Impuls la dezarmare și limită inferioară a ieșirilor după mixer (us).
const int PWM_MAX = 2000;
const int FLIGHT_MIN = 1400; // Baza minimă de throttle în starea armată (us), înaintea corecțiilor PID.
const int FLIGHT_MAX = 1750; // Baza maximă (us); până la PWM_MAX rămân 250 us pentru corecții.

// ---------- Senzor inerțial MPU6050 și magistrala I2C ----------
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int MPU_ADDR = 0x68;

const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_CONFIG       = 0x1A; // Registrul filtrului digital trece-jos (DLPF).
const uint8_t REG_ACCEL_XOUT_H = 0x3B; // Începutul blocului de 14 octeți: accelerație, temperatură, giroscop.

// Offset-uri de calibrare scăzute din citirile brute ale acestui senzor.
// Valorile depind de exemplarul MPU6050 și de calibrarea montajului.
const float OFFSET_ACC_X  = 314.45;
const float OFFSET_ACC_Y  = 422.15;
const float OFFSET_ACC_Z  = -1320.70;
const float OFFSET_GYRO_X = -83.94;
const float OFFSET_GYRO_Y = 318.31;
const float OFFSET_GYRO_Z = 38.81;

const float GYRO_SENS = 131.0; // Sensibilitate: 131 LSB/(grad/s), pentru domeniul de +/-250 grade/s.
const float COMPL_TAU = 0.5;   // Constanta de timp a filtrului complementar, în secunde; alpha depinde de dt.

float pitch = 0.0;
float roll  = 0.0;
float pitchRate = 0.0; // gx, în grade/s; folosit pentru termenul D al buclei Pitch.
float rollRate  = 0.0; // gy, în grade/s; folosit pentru termenul D al buclei Roll.
float yawRate = 0.0; // gz, în grade/s; viteza Yaw, fără integrare într-un unghi.
unsigned long lastFilterTime = 0;

// ---------- Detectarea erorilor de citire IMU ----------
int imuFailCount = 0;
const int IMU_FAIL_LIMIT = 10; // Numărul de citiri consecutive eșuate care blochează activarea ieșirilor.
bool imuHealthy = true;        // Odată setat la false, rămâne false până la repornire.

// ---------- Comenzi primite prin ESP-NOW ----------
// Telecomanda trebuie să trimită aceeași structură binară: aceeași ordine,
// aceleași tipuri și aceeași aliniere a câmpurilor. Throttle/Yaw/Pitch/Roll
// conțin valorile brute ale comenzilor; armed reprezintă comutatorul de armare.
typedef struct struct_message {
  int throttle;
  int yaw;
  int pitch;
  int roll;
  bool armed;
} struct_message;

// Callback-ul ESP-NOW și loop() folosesc aceeași secțiune critică pentru
// copierea structurii și a momentului recepției. Aceasta previne citirea
// unui pachet parțial actualizat; volatile nu asigură singur sincronizarea.
volatile struct_message receivedData;
volatile unsigned long lastPacketTime = 0;
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
const unsigned long FAILSAFE_TIMEOUT_MS = 500;

// ---------- Conversia comenzilor în valori țintă (setpoint-uri) ----------
// Centrele comenzilor, în unități brute, sunt calibrate separat pentru fiecare axă.
// Pentru throttle, centrul marchează începutul creșterii comenzii de bază.
const int CENTER_THROTTLE = 1880;
const int CENTER_YAW      = 1879;
const int CENTER_PITCH    = 1929;
const int CENTER_ROLL     = 1845;

// Intervale de scalare de la centru către capătul cursei, în unități brute.
// Toate sunt configurate aici la 1900; denumirile separate permit calibrarea
// fiecărei comenzi. Pentru throttle se folosește numai cursa deasupra centrului.
const int THROTTLE_HALF_RANGE = 1900;
const int YAW_HALF_RANGE      = 1900;
const int PITCH_HALF_RANGE    = 1900;
const int ROLL_HALF_RANGE     = 1900;

const float MAX_ANGLE = 30.0;       // Limita unghiurilor țintă Pitch și Roll, în grade.
const float MAX_YAW_RATE = 90.0;    // Limita vitezei țintă Yaw, în grade/s.
const int DEADBAND_STICK = 15;      // Abaterile Pitch/Roll/Yaw mai mici de 15 unități față de centru sunt ignorate.

// Marja de armare față de CENTER_THROTTLE, în unități brute.
// Condiția este throttle <= centru + marjă, nu poziția minimă a cursei.
const int ARM_THROTTLE_MARGIN = 50;

// ---------- Control PID pentru unghiurile Pitch și Roll ----------
// Cele două axe folosesc aceiași coeficienți în această implementare.
// P răspunde la eroarea de unghi, I acumulează eroarea în timp, iar D folosește
// viteza măsurată de giroscop. Coeficienții sunt specifici configurației dronei.
const float KP = 1.4;
const float KI = 0.4; // Ponderea integralei; acumularea este limitată la +/-100 în computePID().
const float KD = 0.25;

// Limite individuale ale corecțiilor, aplicate înainte de mixer.
// Reduc contribuția maximă a fiecărei axe, dar nu garantează evitarea
// saturației: suma corecțiilor poate depăși intervalul disponibil al unui ESC.
const float MAX_CORRECTION_PITCH_ROLL = 180.0; // Corecție maximă Pitch/Roll, în us.
const float MAX_CORRECTION_YAW = 200.0;        // Corecție maximă Yaw, în us.

float pitchIntegral = 0.0, rollIntegral = 0.0;

// ---------- Controlul vitezei Yaw ----------
// Cu KD_YAW = 0, bucla Yaw funcționează ca un regulator PI.
const float KP_YAW = 0.4; // Ponderea erorii dintre viteza Yaw cerută și cea măsurată.
const float KI_YAW = 0.05;
const float KD_YAW = 0.0; // Termenul D este dezactivat; apelul pentru Yaw transmite și actualRate = 0.

float yawIntegral = 0.0;

unsigned long lastPidTime = 0;

// ---------- Monitorizarea opțională a bateriei ----------
// Necesită un divizor rezistiv și calibrarea măsurării ADC.
// În această implementare, tensiunea mică blochează o armare nouă și este
// afișată în diagnosticul serial; nu oprește o dronă deja armată.
const bool BATTERY_MONITOR_ENABLED = false;
const int PIN_VBAT = 34; // Intrare ADC pentru divizor; bateria nu se conectează direct la pin.
const float VBAT_ADC_REF = 3.3;
const int VBAT_ADC_MAX = 4095;
const float VBAT_DIVIDER_RATIO = 11.0; // Raportul (R1 + R2) / R2 al divizorului; trebuie să corespundă montajului.
const float VBAT_CELL_COUNT = 3.0;     // Numărul de celule în serie folosit la calculul pragului.
const float VBAT_WARN_PER_CELL = 3.5;  // Prag în V/celulă; se verifică tensiunea totală, nu fiecare celulă separat.

bool batteryLow = false;

// Scrie un registru MPU6050 și returnează true dacă transmisia I2C reușește.
bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

// Selectează registrul de început fără STOP, apoi citește blocul cerut.
// Returnează false dacă selectarea registrului eșuează sau sosesc prea puțini octeți.
bool readRegisters(uint8_t startReg, uint8_t* buffer, uint8_t length) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(MPU_ADDR, (int)length);
  if (Wire.available() < length) return false;
  for (uint8_t i = 0; i < length; i++) buffer[i] = Wire.read();
  return true;
}

// Estimează tensiunea totală din citirea ADC și raportul divizorului.
// Conversia folosește o referință nominală; nu include o calibrare a ADC-ului.
float readBatteryVoltage() {
  int raw = analogRead(PIN_VBAT);
  float vAdc = (raw / (float)VBAT_ADC_MAX) * VBAT_ADC_REF;
  return vAdc * VBAT_DIVIDER_RATIO;
}

void updateFilter() {
  // Citește într-un singur bloc cei 14 octeți ai registrelor 0x3B-0x48:
  // 6 pentru accelerație, 2 pentru temperatură și 6 pentru giroscop.
  // Citirea comună evită tranzacții separate pentru cele două tipuri de măsurători.
  uint8_t buf[14];

  if (!readRegisters(REG_ACCEL_XOUT_H, buf, 14)) {
    imuFailCount++;
    if (imuFailCount >= IMU_FAIL_LIMIT) imuHealthy = false; // Memorează eroarea după atingerea limitei de eșecuri consecutive.
    return; // Păstrează ultimele valori; ieșirile sunt blocate doar după atingerea limitei.
  }
  imuFailCount = 0; // Resetează contorul de eșecuri, fără să reactiveze imuHealthy.

  float ax = ((int16_t)((buf[0] << 8) | buf[1])) - OFFSET_ACC_X;
  float ay = ((int16_t)((buf[2] << 8) | buf[3])) - OFFSET_ACC_Y;
  float az = ((int16_t)((buf[4] << 8) | buf[5])) - OFFSET_ACC_Z;
  // Octeții 6 și 7 conțin temperatura; nu sunt folosiți în acest filtru.

  float gx = (((int16_t)((buf[8]  << 8) | buf[9]))  - OFFSET_GYRO_X) / GYRO_SENS;
  float gy = (((int16_t)((buf[10] << 8) | buf[11])) - OFFSET_GYRO_Y) / GYRO_SENS;
  float gz = (((int16_t)((buf[12] << 8) | buf[13])) - OFFSET_GYRO_Z) / GYRO_SENS;

  // Calculează timpul dintre actualizări folosind rezoluția micros(), apoi
  // îl convertește în secunde. Intervalele nule sau peste 0,5 s devin 0,001 s.
  // dt este folosit pentru integrarea giroscopului și calculul lui alpha.
  unsigned long now = micros();
  float dt = (now - lastFilterTime) / 1000000.0;
  lastFilterTime = now;
  if (dt <= 0 || dt > 0.5) dt = 0.001;

  float pitchAcc = atan2(ay, az) * 180.0 / PI;
  float rollAcc  = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // Filtrul combină unghiul prezis din giroscop cu estimarea accelerometrului.
  // alpha = tau / (tau + dt) adaptează ponderea la intervalul dintre citiri.
  // Accelerațiile de translație pot influența estimarea obținută din accelerometru.
  float alpha = COMPL_TAU / (COMPL_TAU + dt);
  pitch = alpha * (pitch + gx * dt) + (1.0 - alpha) * pitchAcc;
  roll  = alpha * (roll + gy * dt) + (1.0 - alpha) * rollAcc;

  // Păstrează vitezele giroscopului pentru termenul D al controlului de unghi.
  // Acesta folosește mișcarea măsurată, fără derivarea salturilor setpoint-ului.
  pitchRate = gx;
  rollRate  = gy;

  // Viteza Yaw păstrează semnul măsurării gz după corecția de offset.
  // Corespondența dintre semn și sensul fizic depinde de orientarea senzorului.
  // Sensul rotoarelor nu este sensul cuplului lor de reacție asupra cadrului;
  // concordanța dintre axe, sensurile motoarelor și mixer este specifică montajului.
  yawRate = gz; // Fără estimarea unui unghi Yaw; măsurarea include filtrarea DLPF a senzorului.
}

// Regulator cu derivativă pe măsurare: ieșire = kp*eroare + ki*integrală - kd*actualRate.
// actualRate reprezintă derivata mărimii actual: pentru unghiuri, viteza unghiulară.
// Integrala este limitată la +/-100; aceasta este o limitare a acumulării,
// fără compensarea explicită a saturației ieșirilor după mixer.
// Pentru Yaw, actual este deja o viteză: derivata sa ar fi accelerația unghiulară.
// Apelul actual pentru Yaw transmite 0, astfel că termenul D rămâne nul.
float computePID(float setpoint, float actual, float actualRate, float kp, float ki, float kd,
                  float &integral, float dt) {
  float error = setpoint - actual;

  integral += error * dt;
  if (integral > 100) integral = 100;
  if (integral < -100) integral = -100;

  float derivative = -actualRate;

  return (kp * error) + (ki * integral) + (kd * derivative);
}

// Copiază pachetul recepționat și actualizează timpul folosit de failsafe.
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len != sizeof(struct_message)) return; // Acceptă doar dimensiunea așteptată; valorile și expeditorul nu sunt verificate aici.
  portENTER_CRITICAL(&dataMux);
  memcpy((void*)&receivedData, incomingData, sizeof(struct_message));
  lastPacketTime = millis();
  portEXIT_CRITICAL(&dataMux);
}

void setup() {
  pinMode(PIN_LED, OUTPUT);

  // Configurează ieșirile ESC înainte de inițializările și pauzele următoare.
  // Astfel, semnalul PWM_MIN este disponibil devreme în setup(); această
  // secvență nu controlează starea pinilor din timpul pornirii anterioare setup().
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Configurează impulsuri la 250 Hz, adică o perioadă de 4 ms.
  // Aceasta este frecvența semnalului ESC, nu o frecvență impusă buclei PID.
  // ESC-urile utilizate trebuie să accepte această frecvență și duratele de impuls.
  escM1.setPeriodHertz(250); escM1.attach(PIN_ESC_M1, PWM_MIN, PWM_MAX);
  escM2.setPeriodHertz(250); escM2.attach(PIN_ESC_M2, PWM_MIN, PWM_MAX);
  escM3.setPeriodHertz(250); escM3.attach(PIN_ESC_M3, PWM_MIN, PWM_MAX);
  escM4.setPeriodHertz(250); escM4.attach(PIN_ESC_M4, PWM_MIN, PWM_MAX);

  escM1.writeMicroseconds(PWM_MIN);
  escM2.writeMicroseconds(PWM_MIN);
  escM3.writeMicroseconds(PWM_MIN);
  escM4.writeMicroseconds(PWM_MIN);

  Serial.begin(115200);
  delay(500);

  // Inițializează I2C la 400 kHz și scrie configurația MPU6050.
  // writeRegister() returnează starea tranzacției, dar rezultatul nu este
  // verificat în setup(); imuHealthy urmărește erorile de citire ulterioare.
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  writeRegister(REG_PWR_MGMT_1, 0x00);   // Ieșire din modul sleep; selectează oscilatorul intern.
  delay(10);
  writeRegister(REG_GYRO_CONFIG, 0x00);  // Domeniu giroscop: +/-250 grade/s, corespunzător GYRO_SENS = 131.
  writeRegister(REG_ACCEL_CONFIG, 0x00); // Domeniu accelerometru: +/-2 g.
  writeRegister(REG_CONFIG, 0x04);       // DLPF_CFG = 4: bandă de 21 Hz pentru accelerație și 20 Hz pentru giroscop.
  delay(100);

  if (BATTERY_MONITOR_ENABLED) {
    analogReadResolution(12);
  }

  delay(2500); // Pauză suplimentară de inițializare a ESC-urilor, menținând PWM_MIN.

  // Activează modul Wi-Fi STA și înregistrează callback-ul de recepție ESP-NOW.
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("EROARE: esp_now_init() a esuat.");
  } else {
    esp_now_register_recv_cb(OnDataRecv);
  }

  lastFilterTime = micros();
  lastPidTime = micros();

  Serial.println("=== DRONA - cod complet PID + mixer pornit ===");
  Serial.println("ATENTIE: motoarele sunt active. Fara elice montate!");
}

void loop() {
  updateFilter();

  // Comută LED-ul fără delay(). Indică rularea buclei, nu starea de armare
  // sau validitatea senzorului. Un ciclu complet durează aproximativ o secundă.
  static unsigned long lastLedToggle = 0;
  static bool ledState = false;
  if (millis() - lastLedToggle >= LED_BLINK_INTERVAL_MS) {
    lastLedToggle = millis();
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState ? HIGH : LOW);
  }

  // Preia împreună ultimul pachet și momentul recepției într-o copie locală.
  // Secțiunea critică împiedică modificarea lor de către callback în timpul copierii.
  struct_message localData;
  unsigned long localLastPacketTime;
  portENTER_CRITICAL(&dataMux);
  memcpy(&localData, (const void*)&receivedData, sizeof(struct_message));
  localLastPacketTime = lastPacketTime;
  portEXIT_CRITICAL(&dataMux);

  if (BATTERY_MONITOR_ENABLED) {
    static unsigned long lastVbatCheck = 0;
    if (millis() - lastVbatCheck > 1000) {
      lastVbatCheck = millis();
      float vbat = readBatteryVoltage();
      batteryLow = (vbat < VBAT_CELL_COUNT * VBAT_WARN_PER_CELL);
    }
  }

  // Failsafe activ înaintea primei recepții sau după mai mult de 500 ms fără pachete.
  bool failsafe = (localLastPacketTime == 0) || (millis() - localLastPacketTime > FAILSAFE_TIMEOUT_MS);

  // ---------- Armare la tranziția observată a comutatorului ----------
  // Memorează armarea numai la trecerea oprit -> pornit observată de loop(),
  // cu throttle <= centru + marjă și, dacă monitorizarea este activă,
  // fără avertizare de baterie slabă. Dacă aceste condiții nu sunt îndeplinite,
  // este necesară o nouă tranziție a comutatorului după îndeplinirea lor.
  // Oprirea comutatorului șterge starea memorată în armedLatch.
  static bool armSwitchPrev = false;
  static bool armedLatch = false;
  bool throttleLow = localData.throttle <= (CENTER_THROTTLE + ARM_THROTTLE_MARGIN);

  if (localData.armed && !armSwitchPrev) {
    if (throttleLow && (!BATTERY_MONITOR_ENABLED || !batteryLow)) {
      armedLatch = true;
    }
  }
  if (!localData.armed) {
    armedLatch = false;
  }
  armSwitchPrev = localData.armed;

  // Activează comenzile numai dacă armarea este memorată, legătura este recentă
  // și nu există o eroare IMU memorată. Failsafe-ul nu resetează armedLatch:
  // revenirea pachetelor poate reactiva comenzile fără o nouă tranziție de armare.
  bool armed = armedLatch && !failsafe && imuHealthy;

  int m1, m2, m3, m4;

  if (!armed) {
    // Dezactivare: trimite PWM_MIN și șterge integralele tuturor celor trei axe.
    m1 = m2 = m3 = m4 = PWM_MIN;
    pitchIntegral = rollIntegral = yawIntegral = 0;
  } else {
    // Calculează abaterile față de centre și aplică zona moartă pe Pitch/Roll/Yaw.
    int pitchRaw = localData.pitch - CENTER_PITCH;
    int rollRaw  = localData.roll  - CENTER_ROLL;
    int yawRaw   = localData.yaw   - CENTER_YAW;
    if (abs(pitchRaw) < DEADBAND_STICK) pitchRaw = 0;
    if (abs(rollRaw)  < DEADBAND_STICK) rollRaw  = 0;
    if (abs(yawRaw)   < DEADBAND_STICK) yawRaw   = 0;

    // Convertește abaterile în unghiuri țintă. Semnul minus al comenzii Pitch
    // face parte din convenția aleasă pentru telecomandă și orientarea senzorului.
    float pitchSetpoint = -((float)pitchRaw / PITCH_HALF_RANGE) * MAX_ANGLE;
    float rollSetpoint  = ((float)rollRaw / ROLL_HALF_RANGE) * MAX_ANGLE;
    // Convertește comanda Yaw într-o viteză țintă, cu semnul inversat.
    // Acest semn privește comanda telecomenzii; yawRate rămâne măsurarea gz.
    // Setpoint-urile sunt apoi limitate la intervalele configurate.
    float yawRateSetpoint = -((float)yawRaw / YAW_HALF_RANGE) * MAX_YAW_RATE;
    pitchSetpoint = constrain(pitchSetpoint, -MAX_ANGLE, MAX_ANGLE);
    rollSetpoint  = constrain(rollSetpoint, -MAX_ANGLE, MAX_ANGLE);
    yawRateSetpoint = constrain(yawRateSetpoint, -MAX_YAW_RATE, MAX_YAW_RATE);

    unsigned long now = micros(); // Timpul dintre calculele PID; actualizat numai în ramura armată.
    float dt = (now - lastPidTime) / 1000000.0;
    lastPidTime = now;
    if (dt <= 0 || dt > 0.5) dt = 0.001;

    // Calculează baza impulsurilor ESC înainte de adăugarea corecțiilor.
    // La centru sau sub centru: FLIGHT_MIN (1400 us). Deasupra centrului:
    // creștere liniară până la FLIGHT_MAX (1750 us), limitată la cursa configurată.
    // Valorile individuale ale motoarelor pot coborî sub FLIGHT_MIN după mixer.
    int baseThrottle;
    if (localData.throttle <= CENTER_THROTTLE) {
      baseThrottle = FLIGHT_MIN;
    } else {
      int aboveCenter = constrain(localData.throttle - CENTER_THROTTLE, 0, THROTTLE_HALF_RANGE);
      baseThrottle = map(aboveCenter, 0, THROTTLE_HALF_RANGE, FLIGHT_MIN, FLIGHT_MAX);
    }

    // La bază mică de throttle (cel mult FLIGHT_MIN + 20 us), șterge integralele
    // înaintea fiecărui calcul PID. computePID() adaugă apoi eroarea pasului curent;
    // prin urmare, acumularea între iterații este eliminată, nu și termenul I complet.
    // Condiția folosește doar throttle-ul; nu detectează contactul cu solul.
    if (baseThrottle <= FLIGHT_MIN + 20) {
      pitchIntegral = 0;
      rollIntegral = 0;
      yawIntegral = 0;
    }

    float pitchCorrection = computePID(pitchSetpoint, pitch, pitchRate, KP, KI, KD, pitchIntegral, dt);
    float rollCorrection  = computePID(rollSetpoint, roll, rollRate, KP, KI, KD, rollIntegral, dt);
    // Yaw este controlat ca viteză. Cu KD_YAW = 0 și actualRate = 0,
    // corecția rezultă numai din termenii P și I.
    float yawCorrection   = computePID(yawRateSetpoint, yawRate, 0.0, KP_YAW, KI_YAW, KD_YAW, yawIntegral, dt);

    // Limitează fiecare corecție înainte de combinare. După mixer, fiecare
    // ieșire este limitată separat; o eventuală saturație poate altera proporțiile.
    pitchCorrection = constrain(pitchCorrection, -MAX_CORRECTION_PITCH_ROLL, MAX_CORRECTION_PITCH_ROLL);
    rollCorrection  = constrain(rollCorrection, -MAX_CORRECTION_PITCH_ROLL, MAX_CORRECTION_PITCH_ROLL);
    yawCorrection   = constrain(yawCorrection, -MAX_CORRECTION_YAW, MAX_CORRECTION_YAW);

    // Mixer pentru configurația X: adaugă sau scade corecțiile pe fiecare motor.
    // Pitch: față/spate; Roll: stânga/dreapta; Yaw: grupurile M1+M4 și M2+M3.
    // Semnele descriu acest mixer și trebuie interpretate împreună cu montajul.
    // lroundf() rotunjește corecțiile la cel mai apropiat întreg, în microsecunde,
    // în loc să elimine partea fracționară prin trunchiere spre zero.
    long pc = lroundf(pitchCorrection);
    long rc = lroundf(rollCorrection);
    long yc = lroundf(yawCorrection);

    m1 = baseThrottle + pc + rc + yc; // M1: față-stânga, CCW.
    m2 = baseThrottle + pc - rc - yc; // M2: față-dreapta, CW.
    m3 = baseThrottle - pc + rc - yc; // M3: spate-stânga, CW.
    m4 = baseThrottle - pc - rc + yc; // M4: spate-dreapta, CCW.

    // Limitează duratele finale la intervalul PWM_MIN..PWM_MAX (1000..2000 us).
    m1 = constrain(m1, PWM_MIN, PWM_MAX);
    m2 = constrain(m2, PWM_MIN, PWM_MAX);
    m3 = constrain(m3, PWM_MIN, PWM_MAX);
    m4 = constrain(m4, PWM_MIN, PWM_MAX);
  }

  // Transmite cele patru durate de impuls către ieșirile ESC.
  escM1.writeMicroseconds(m1);
  escM2.writeMicroseconds(m2);
  escM3.writeMicroseconds(m3);
  escM4.writeMicroseconds(m4);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 50) { // Diagnostic la intervale de peste 50 ms, aproximativ 20 de afișări/s.
    lastPrint = millis();
    Serial.print("Armed: "); Serial.print(armed ? "DA" : "nu");
    Serial.print(!imuHealthy ? " [IMU EROARE]" : "");
    Serial.print(failsafe ? " [FAILSAFE]" : "");
    Serial.print((BATTERY_MONITOR_ENABLED && batteryLow) ? " [BATERIE SLABA]" : "");
    Serial.print("  Pitch: "); Serial.print(pitch, 1);
    Serial.print("  Roll: "); Serial.print(roll, 1);
    Serial.print("  YawRate: "); Serial.print(yawRate, 1);
    Serial.print("  | M1:"); Serial.print(m1);
    Serial.print(" M2:"); Serial.print(m2);
    Serial.print(" M3:"); Serial.print(m3);
    Serial.print(" M4:"); Serial.println(m4);
  }
}