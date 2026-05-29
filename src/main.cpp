#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <driver/adc.h>   // doc ADC nhanh hon analogRead

/*
  Robot do line Cascade PD + Gyro P
  Phuong phap: cascade nhu code mau - line PD tinh target turn rate,
               gyro P bu sai so turn rate thuc, cho phan hoi nhanh + on dinh.

  Phan cung:
    - ESP32 + DRV8833 (STBY noi 3V3)
    - 8 cam bien line analog (trai->phai: GPIO36,39,34,35,32,33,25,26)
    - MPU6050 I2C (SDA=21, SCL=22)
    - 3x VL53L0X XSHUT (trai=17, truoc=16, phai=23) cho maze
*/

// ================================================================
//  CHAN - SUA THEO MACH THAT
// ================================================================

const int SDA_PIN = 21;
const int SCL_PIN = 22;

// 8 cam bien line: thu tu TU TRAI SANG PHAI khi nhin theo chieu xe chay
const int NUM_SENSORS = 8;
const int sensorPins[NUM_SENSORS] = { 36, 39, 34, 35, 32, 33, 25, 26 };

// Nguong rieng tung cam bien (chinh thuc te qua Serial)
// Gia tri analogRead < threshold = MAU DEN (detect line)
int thresholds[NUM_SENSORS] = { 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000 };

// DRV8833 (STBY -> 3V3, khong can pin dieu khien)
const int L_IN1 = 18;   // Motor trai
const int L_IN2 = 19;
const int R_IN1 = 27;   // Motor phai
const int R_IN2 = 14;

// VL53L0X XSHUT cho maze
const int XSHUT_L = 17;
const int XSHUT_F = 16;
const int XSHUT_R = 23;
const uint8_t ADDR_L = 0x30;
const uint8_t ADDR_F = 0x31;
const uint8_t ADDR_R = 0x32;

// *** CHAN CHON CHE DO ***
// D4 (GPIO4): dung INPUT_PULLUP
//   - Cam jumper/day tu D4 xuong GND  -> LOC (DO LINE)
//   - Rut day ra, pin thả nổi -> HIGH  -> MAZE (GIAI ME CUNG)
const int MODE_PIN = 4;

// ================================================================
//  THAM SO DIEU KHIEN - chinh o day
// ================================================================

// --- Cascade PD (line) + Gyro P (rate) ---
// lineKp/lineKd: quy doi error vi tri (don vi: 0..3500) -> target turn rate (rad/s)
// gyroKp:        quy doi sai so turn rate -> PWM correction
float lineKp = 0.0021f;
float lineKd = 0.0075f;
float gyroKp = 65.0f;     // giam tu 72 -> tranh over-correction

// --- Toc do ---
int fullSpeed   = 255;
int fastSpeed   = 255;
int midSpeed    = 255;
int curveSpeed  = 255;
int sharpSpeed  = 255;

int currentSpeed  = 255;
int speedStepUp   = 255;
int speedStepDown = 60;   // phanh manh hon

// --- Loc EMA cho error ---
float filteredError = 0.0f;
float errorAlpha    = 1.0f;  // 1.0 = khong loc, phan hoi NGAY LAP TUC voi moi thay doi

// --- Tim line khi mat ---
int searchBackSpeed = 255; // lui max de tim lai line khi bi qua vach
const unsigned long SEARCH_BACK_MS = 400;

// --- Maze ---
bool mazeEnabled = false;  // doi sang true khi san sang test maze
int  baseMazeSpeed   = 153;
int  turnMazeSpeed   = 140;

// --- Nguong cam bien maze ---
// Cam bien TRAI va PHAI gac 45° so voi truc xe:
//   Layout (nhin tu tren):   L\  /R
//                              [F]
// Khi co tuong ca 2 phia, cam bien 45° doc duong cheo:
//   d_sensor = d_wall / cos(45deg) = d_wall * 1.414
// => Nguong cho cam bien 45° phai lon hon ~1.4x cam bien thang
// WALL_DIAG_MM: nguong de phat hien "co lo" phia trai/phai (chinh theo thuc te)
const int WALL_DIAG_MM = 280;  // cam bien 45°: > nguong nay = co khoang mo o goc 45°
const int FRONT_MM     = 180;  // cam bien thang truoc: > nguong nay = duong truoc thong
const int CRASH_MM     =  80;  // dung khan cap neu vat qua gan phia truoc
const float WALL_KP    = 0.15f; // he so chinh lane dung cam bien 45°
const unsigned long MAZE_FWD_MS = 280;

// ================================================================
//  BIEN TOAN CUC
// ================================================================
Adafruit_MPU6050 mpu;
Adafruit_VL53L0X vl53L, vl53F, vl53R;

bool mpuReady  = false;
bool vl53Ready = false;

bool sensorBlack[NUM_SENSORS];
bool lineDetected  = false;
int  lastError     = 0;

float gyroZOffset = 0.0f;

enum RobotMode { LINE_FOLLOW, MAZE_SOLVE };
RobotMode mode = LINE_FOLLOW;

// --- Cache gyro ---
float cachedGyroZ  = 0.0f;
int   gyroSkipCnt  = 0;       // doc gyro moi 3 loop de tang tan so quet sensor

// ================================================================
//  MOTOR - DRV8833
// ================================================================
void setupMotors() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // Arduino ESP32 v3+: analogWrite tren bat ky chan nao
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
#else
  ledcSetup(0, 20000, 8); ledcAttachPin(L_IN1, 0);
  ledcSetup(1, 20000, 8); ledcAttachPin(L_IN2, 1);
  ledcSetup(2, 20000, 8); ledcAttachPin(R_IN1, 2);
  ledcSetup(3, 20000, 8); ledcAttachPin(R_IN2, 3);
#endif
}

void writeMotor(int in1, int in2, int ch1, int ch2, int speed) {
  speed = constrain(speed, -255, 255);
  int a = (speed > 0) ? speed : 0;
  int b = (speed < 0) ? -speed : 0;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  analogWrite(in1, a);
  analogWrite(in2, b);
#else
  ledcWrite(ch1, a);
  ledcWrite(ch2, b);
#endif
}

// speed > 0 = tien, speed < 0 = lui
void setMotorLeft(int speed)  { writeMotor(L_IN1, L_IN2, 0, 1, speed); }
void setMotorRight(int speed) { writeMotor(R_IN1, R_IN2, 2, 3, speed); }

void setMotors(int l, int r) { setMotorLeft(l); setMotorRight(r); }
void stopMotors()             { setMotorLeft(0); setMotorRight(0); }

// ================================================================
//  GYRO
// ================================================================
void calibrateGyro() {
  Serial.println("Calibrating gyro... giu robot dung yen");
  float sum = 0;
  const int N = 500;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sum += g.gyro.z;
    delay(2);
  }
  gyroZOffset = sum / N;
  Serial.printf("gyroZOffset = %.6f\n", gyroZOffset);
}

float readGyroZ() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  return g.gyro.z - gyroZOffset; // rad/s
}

// ================================================================
//  CAM BIEN LINE
// ================================================================
/*
  ADC channel map cho ESP32 ADC1 (dung adc1_get_raw thay analogRead, nhanh hon ~3x):
  GPIO36=ADC1_CH0, GPIO39=ADC1_CH3, GPIO34=ADC1_CH6, GPIO35=ADC1_CH7
  GPIO32=ADC1_CH4, GPIO33=ADC1_CH5, GPIO25=ADC1_CH8(*), GPIO26=ADC1_CH9(*)
  (*) GPIO25/26 la ADC2 nen van dung analogRead cho 2 sensor cuoi
*/
static const adc1_channel_t adc1ch[6] = {
  ADC1_CHANNEL_0,  // GPIO36
  ADC1_CHANNEL_3,  // GPIO39
  ADC1_CHANNEL_6,  // GPIO34
  ADC1_CHANNEL_7,  // GPIO35
  ADC1_CHANNEL_4,  // GPIO32
  ADC1_CHANNEL_5,  // GPIO33
};

void setupADC() {
  adc1_config_width(ADC_WIDTH_BIT_12);
  for (int i = 0; i < 6; i++)
    adc1_config_channel_atten(adc1ch[i], ADC_ATTEN_DB_12); // 0..3.3V, DB_11 deprecated
}

int fastRead(int sensorIdx) {
  if (sensorIdx < 6) return adc1_get_raw(adc1ch[sensorIdx]);
  return analogRead(sensorPins[sensorIdx]);
}

int readLineError() {
  long weightedSum = 0;
  int  activeCount = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = fastRead(i);
    sensorBlack[i] = (val < thresholds[i]);
    if (sensorBlack[i]) {
      weightedSum += (long)i * 1000;
      activeCount++;
    }
  }

  lineDetected = (activeCount > 0);

  if (lineDetected) {
    int position = (int)(weightedSum / activeCount);
    int err = position - 3500;

    return err;
  }

  return lastError; // giu error cu khi mat line
}

int countBlack() {
  int n = 0;
  for (int i = 0; i < NUM_SENSORS; i++) if (sensorBlack[i]) n++;
  return n;
}

// ================================================================
//  PHAT HIEN GOC VUONG BANG PATTERN
//  Nhan biet ngay khi >= 3 sensor CANH cung den
//  (xuat hien TRUOC KHI error tang du cao)
// ================================================================
bool isSharpCornerPattern() {
  // Canh trai: sensor 0,1,2 - neu >= 2 trong so 3 sensor nay den
  // VA cac sensor ben phai (5,6,7) khong den -> goc trai
  int leftEdge  = (int)sensorBlack[0] + sensorBlack[1] + sensorBlack[2];
  int rightEdge = (int)sensorBlack[5] + sensorBlack[6] + sensorBlack[7];
  int centerOn  = (int)sensorBlack[3] + sensorBlack[4];

  // Goc trai: it nhat 2/3 sensor trai den, trung tam khong den
  if (leftEdge >= 2 && centerOn == 0 && rightEdge == 0) return true;
  // Goc phai: it nhat 2/3 sensor phai den, trung tam khong den
  if (rightEdge >= 2 && centerOn == 0 && leftEdge == 0) return true;
  return false;
}

// ================================================================
//  DYNAMIC SPEED
// ================================================================
void rampSpeedTo(int target) {
  if (currentSpeed < target) {
    currentSpeed = min(currentSpeed + speedStepUp,   target);
  } else if (currentSpeed > target) {
    currentSpeed = max(currentSpeed - speedStepDown, target);
  }
}

int getTargetSpeed(int absError) {
  bool centerBoth = sensorBlack[3] && sensorBlack[4];
  if (centerBoth && absError < 600)  return fullSpeed;  // thang hop
  if (absError < 1100)               return fastSpeed;  // lech nhe
  if (absError < 2000)               return midSpeed;   // cua vua
  if (absError < 2800)               return curveSpeed; // cua gat
  return sharpSpeed;                                    // GOC VUONG 90 do
}

// ================================================================
//  TIM LINE KHI MAT - BLOCKING
//  Khi mat line, khong xoay theo raw error cuoi nua.
//  Lui thang mot doan ngan de dua day sensor ve lai vach vua bi vuot qua.
// ================================================================
void searchLine() {
  Serial.println("[SEARCH] backtrack");

  unsigned long t0 = millis();
  while (millis() - t0 < SEARCH_BACK_MS) {
    readLineError();
    if (lineDetected) goto done;
    setMotors(-searchBackSpeed, -searchBackSpeed);
  }

  Serial.println("[SEARCH] NOT FOUND");
  stopMotors(); delay(200);
  return;

done:
  Serial.println("[SEARCH] FOUND");
  stopMotors(); delay(30);
  filteredError = 0.0f;
  lastError     = 0;
  currentSpeed  = sharpSpeed;
}

void resetSearch() {}

// ================================================================
//  FOLLOW LINE - CASCADE PD + GYRO P
// ================================================================
void followLineCascade() {
  // Doc gyro moi 3 loop thay vi moi loop -> tang tan so quet sensor ~3x
  gyroSkipCnt++;
  if (gyroSkipCnt >= 3) {
    gyroSkipCnt = 0;
    if (mpuReady) {
      sensors_event_t a, g, t;
      mpu.getEvent(&a, &g, &t);
      cachedGyroZ = g.gyro.z - gyroZOffset;
    }
  }

  int rawError = readLineError();

  if (!lineDetected) {
    searchLine();
    return;
  }

  // EMA filter
  filteredError = filteredError * (1.0f - errorAlpha) + rawError * errorAlpha;
  int error      = (int)filteredError;
  int derivative = error - lastError;
  int absError   = abs(error);
  int absRaw     = abs(rawError);

  // Toc do theo error - ramp binh thuong
  rampSpeedTo(getTargetSpeed(absRaw));

  // --- Cascade PD -> target turn rate ---
  float targetTurnRate = lineKp * error + lineKd * derivative;
  targetTurnRate = constrain(targetTurnRate, -3.8f, 3.8f);

  // --- Gyro P ---
  float turnErr  = targetTurnRate + cachedGyroZ;
  int correction = (int)(gyroKp * turnErr);

  // Gioi han correction - CHAT HON de tranh over-correction o goc vuong
  if      (absError > 2600) correction = constrain(correction, -180, 180);
  else if (absError > 1900) correction = constrain(correction, -145, 145);
  else if (absError > 1200) correction = constrain(correction, -115, 115);
  else                      correction = constrain(correction,  -95,  95);

  int leftSpeed  = currentSpeed + correction;
  int rightSpeed = currentSpeed - correction;

  int brakeLow = (absError > 2800) ? -200 : (absError > 1800 ? -130 : -80);
  leftSpeed  = constrain(leftSpeed,  brakeLow, 255);
  rightSpeed = constrain(rightSpeed, brakeLow, 255);

  setMotorLeft(leftSpeed);
  setMotorRight(rightSpeed);

  lastError = error;
}

// ================================================================
//  VL53L0X
// ================================================================
bool setupVL53L0X() {
  pinMode(XSHUT_L, OUTPUT); pinMode(XSHUT_F, OUTPUT); pinMode(XSHUT_R, OUTPUT);
  digitalWrite(XSHUT_L, LOW); digitalWrite(XSHUT_F, LOW); digitalWrite(XSHUT_R, LOW);
  delay(20);

  digitalWrite(XSHUT_L, HIGH); delay(20);
  if (!vl53L.begin(ADDR_L, false, &Wire)) { Serial.println("[LOI] VL53 TRAI"); return false; }

  digitalWrite(XSHUT_F, HIGH); delay(20);
  if (!vl53F.begin(ADDR_F, false, &Wire)) { Serial.println("[LOI] VL53 TRUOC"); return false; }

  digitalWrite(XSHUT_R, HIGH); delay(20);
  if (!vl53R.begin(ADDR_R, false, &Wire)) { Serial.println("[LOI] VL53 PHAI"); return false; }

  Serial.println("[OK] 3x VL53L0X ready");
  return true;
}

int readMM(Adafruit_VL53L0X &s) {
  VL53L0X_RangingMeasurementData_t m;
  s.rangingTest(&m, false);
  return (m.RangeStatus == 4) ? 8190 : (int)m.RangeMilliMeter;
}

// ================================================================
//  MAZE SOLVER - cam bien 45 do
//
//  Bo tri cam bien (nhin tu tren):
//
//       L (45°)\     /(45°) R
//                \ /
//               [===]  <-- F (thang truoc)
//
//  Nguyen tac Left-Hand Rule:
//    Uu tien re TRAI > di THANG > re PHAI > QUAY DAU
//
//  Tai mot nga tu:
//    - Cam bien 45° trai nhin vao hanh lang trai -> doc rat xa (> WALL_DIAG_MM)
//    - Cam bien 45° phai nhin vao hanh lang phai -> doc rat xa (> WALL_DIAG_MM)
//    - Cam bien truoc: phat hien tuong thang truoc (< FRONT_MM = bi chan)
//
//  Trong hanh lang thang:
//    - Ca L va R doc ngan (bi tuong 2 ben chan o goc 45°)
//    - Dung chenh lech L/R de chinh lane (wall-centering)
// ================================================================
float headingDeg     = 0.0f;
unsigned long lastGyroUs = 0;

void updateHeading() {
  unsigned long now = micros();
  static unsigned long last = 0;
  if (last == 0) { last = now; return; }
  float dt = (now - last) * 1e-6f;
  last = now;
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  headingDeg += (g.gyro.z - gyroZOffset) * 57.2957795f * dt;
}

void resetHeading() { headingDeg = 0.0f; lastGyroUs = micros(); }

// Quay tai cho dung deg do (am = trai, duong = phai)
// Giam toc khi con < 20° de khong vuot qua
void turnByDeg(float deg) {
  resetHeading();
  int dir = (deg > 0) ? 1 : -1;
  float target = fabsf(deg);
  unsigned long start = millis();
  while (fabsf(headingDeg) < target && millis() - start < 3000) {
    updateHeading();
    int spd = (target - fabsf(headingDeg) < 20.0f) ? 65 : turnMazeSpeed;
    setMotors(dir * spd, -dir * spd);
    delay(3);
  }
  stopMotors(); delay(100); resetHeading();
}

// Tien thang 1 buoc, dung 2 tang chinh huong:
//   1. Gyro P: giu huong khong xoay
//   2. Wall centering: can bang L/R de di giua hanh lang
void mazeFwd() {
  resetHeading();
  unsigned long start = millis();
  while (millis() - start < MAZE_FWD_MS) {
    updateHeading();
    if (readMM(vl53F) < CRASH_MM) break;  // dung khan cap

    // Tang 1: chinh thang bang gyro (chong xoay)
    float gyroCorr = headingDeg * 1.5f;

    // Tang 2: wall-centering bang cam bien 45°
    // Chi ap dung khi ca 2 cam bien thay tuong (trong hanh lang, khong tai nga tu)
    int lMM = readMM(vl53L);
    int rMM = readMM(vl53R);
    float wallCorr = 0.0f;
    if (lMM < WALL_DIAG_MM && rMM < WALL_DIAG_MM) {
      // lMM < rMM: robot lech sang phai (gan tuong phai) -> can sang trai
      // lMM > rMM: robot lech sang trai (gan tuong trai) -> can sang phai
      wallCorr = (float)(lMM - rMM) * WALL_KP;
    }

    float totalCorr = gyroCorr + wallCorr;
    setMotors(baseMazeSpeed - (int)totalCorr,
              baseMazeSpeed + (int)totalCorr);
    delay(4);
  }
  stopMotors(); delay(80);
}

// Quyet dinh huong di tai moi buoc (Left-Hand Rule)
void solveMaze() {
  if (!vl53Ready) { stopMotors(); delay(300); return; }

  int l = readMM(vl53L);
  int f = readMM(vl53F);
  int r = readMM(vl53R);
  Serial.printf("[MAZE] L=%d F=%d R=%d | thresh_diag=%d front=%d\n",
                l, f, r, WALL_DIAG_MM, FRONT_MM);

  // LEFT-HAND RULE voi cam bien 45°:
  // Cam bien 45° trai doc xa -> hanh lang trai mo -> re trai
  if (l > WALL_DIAG_MM) {
    Serial.println("[MAZE] -> RE TRAI");
    turnByDeg(-90);
    mazeFwd();
  }
  // Truoc thong -> tien thang
  else if (f > FRONT_MM) {
    Serial.println("[MAZE] -> THANG");
    mazeFwd();
  }
  // Cam bien 45° phai doc xa -> hanh lang phai mo -> re phai
  else if (r > WALL_DIAG_MM) {
    Serial.println("[MAZE] -> RE PHAI");
    turnByDeg(90);
    mazeFwd();
  }
  // Bit ca 3 huong -> quay dau
  else {
    Serial.println("[MAZE] -> QUAY DAU");
    turnByDeg(180);
  }
}

// ================================================================
//  SETUP / LOOP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(800000); // 800kHz: nhanh hon 2x, MPU6050 ho tro den 1MHz

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 NOT FOUND!");
  } else {
    Serial.println("MPU6050 CONNECTED!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    calibrateGyro();
    mpuReady = true;
  }

  for (int i = 0; i < NUM_SENSORS; i++) pinMode(sensorPins[i], INPUT);

  // Cau hinh chan chon che do: pull-up noi, cam GND = LINE, rut ra = MAZE
  pinMode(MODE_PIN, INPUT_PULLUP);

  setupMotors();
  setupADC(); // khoi tao ADC1 nhanh cho 6 sensor dau

  // VL53L0X chi can khi maze bat (init de san sang)
  vl53Ready = setupVL53L0X();
  if (!vl53Ready) Serial.println("[WARN] VL53 loi, chi chay line mode");

  analogReadResolution(12);

  // Doc trang thai ban dau cua MODE_PIN
  mode = (digitalRead(MODE_PIN) == LOW) ? LINE_FOLLOW : MAZE_SOLVE;
  Serial.printf("[BOOT] MODE_PIN=D4 -> %s\n",
                mode == LINE_FOLLOW ? "LINE_FOLLOW" : "MAZE_SOLVE");
  Serial.println("=== START ===");
  Serial.printf("lineKp=%.5f lineKd=%.5f gyroKp=%.1f\n", lineKp, lineKd, gyroKp);
  delay(500);
}

void loop() {
  // Cập nhật mode theo trang thai chan D4 (real-time)
  RobotMode newMode = (digitalRead(MODE_PIN) == LOW) ? LINE_FOLLOW : MAZE_SOLVE;
  if (newMode != mode) {
    mode = newMode;
    stopMotors();
    filteredError = 0.0f;
    lastError     = 0;
    Serial.printf("[MODE] Chuyen sang: %s\n",
                  mode == LINE_FOLLOW ? "LINE_FOLLOW" : "MAZE_SOLVE");
    delay(200); // chong nhieu tiep xuc
  }

  if (mode == LINE_FOLLOW) {
    followLineCascade();
  } else {
    solveMaze();
  }
}

