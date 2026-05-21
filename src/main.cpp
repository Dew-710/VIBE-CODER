#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>

/*
  Robot do line va giai me cung dung ESP32
  - 8 cam bien line analog: gia tri < threshold la den, > threshold la trang
  - DRV8833 dieu khien 2 motor DC
  - MPU6050 doc gyro Z de ho tro giu huong va re goc
  - 3 VL53L0X dung chung I2C, tach dia chi bang chan XSHUT

  Luu y:
  - Hay sua lai cac chan ben duoi theo mach that.
  - ESP32 ADC1 gom GPIO 32-39. Neu co the, nen uu tien ADC1 cho cam bien analog.
*/

// ===================== KHAI BAO CHAN DE TU CHINH =====================

// I2C cho MPU6050 va VL53L0X
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

// 8 cam bien line analog, tu trai sang phai
const int NUM_LINE_SENSORS = 8;
const int linePins[NUM_LINE_SENSORS] = {
  36, 39, 34, 35, 32, 33, 25, 26
};

// Threshold rieng tung cam bien, can calib thuc te
int lineThresholds[NUM_LINE_SENSORS] = {
  2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000
};

// DRV8833
const int MOTOR_STBY_PIN = 23;

const int LEFT_AIN1_PIN = 16;
const int LEFT_AIN2_PIN = 17;
const int RIGHT_BIN1_PIN = 18;
const int RIGHT_BIN2_PIN = 19;

// PWM ESP32
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int PWM_LEFT_IN1_CH = 0;
const int PWM_LEFT_IN2_CH = 1;
const int PWM_RIGHT_IN1_CH = 2;
const int PWM_RIGHT_IN2_CH = 3;

// XSHUT cua 3 cam bien VL53L0X
const int VL53_LEFT_XSHUT_PIN = 13;
const int VL53_FRONT_XSHUT_PIN = 14;
const int VL53_RIGHT_XSHUT_PIN = 27;

// Dia chi I2C moi cho tung VL53L0X
const uint8_t VL53_LEFT_ADDR = 0x30;
const uint8_t VL53_FRONT_ADDR = 0x31;
const uint8_t VL53_RIGHT_ADDR = 0x32;

// ===================== THAM SO DIEU KHIEN =====================

// Line following
int baseSpeedLine = 130;
float lineKp = 0.060f;
float lineKd = 0.850f;
float straightGyroKp = 2.0f;   // ho tro giu huong khi line error gan 0

// Maze
int baseSpeedMaze = 120;
int turnSpeedMaze = 115;
const int WALL_DISTANCE_MM = 170;       // nho hon nguong nay xem la co tuong/vat can
const int FRONT_CLEAR_DISTANCE_MM = 190;
const unsigned long MAZE_FORWARD_MS = 280;

// Debug
const unsigned long DEBUG_INTERVAL_MS = 150;

// ===================== BIEN TOAN CUC =====================

enum RobotMode {
  LINE_FOLLOW_MODE,
  MAZE_SOLVE_MODE
};

RobotMode currentMode = LINE_FOLLOW_MODE;

Adafruit_MPU6050 mpu;
Adafruit_VL53L0X vl53Left;
Adafruit_VL53L0X vl53Front;
Adafruit_VL53L0X vl53Right;

bool mpuReady = false;
bool vl53Ready = false;

int lineValues[NUM_LINE_SENSORS];
bool lineIsBlack[NUM_LINE_SENSORS];

float lastLineError = 0.0f;
float targetHeadingDeg = 0.0f;
float currentHeadingDeg = 0.0f;
float gyroZOffsetRad = 0.0f;
unsigned long lastGyroUpdateMs = 0;
unsigned long lastDebugMs = 0;

// ===================== KHAI BAO HAM =====================

void setupMotors();
void setMotorLeft(int speed);
void setMotorRight(int speed);
void setMotors(int leftSpeed, int rightSpeed);
void stopMotors();

bool setupVL53L0X();
int readDistanceMM(Adafruit_VL53L0X &sensor);
void readMazeDistances(int &leftMM, int &frontMM, int &rightMM);

bool setupMPU6050();
void calibrateGyroZ();
void updateGyroHeading();
void resetHeading(float headingDeg = 0.0f);

void readLineSensors();
float calculateLineError();
bool allSensorsBlack();
void followLine();

void solveMaze();
void goForwardMaze();
void turnLeft90();
void turnRight90();
void turnAround180();
void turnByDegrees(float degrees);

void printLineDebug();
void printModeDebug();

// ===================== SETUP / LOOP =====================

void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12); // ESP32: 0..4095

  setupMotors();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  mpuReady = setupMPU6050();
  if (!mpuReady) {
    Serial.println("LOI: Khong tim thay MPU6050. Kiem tra day noi/I2C.");
  } else {
    calibrateGyroZ();
    resetHeading(0.0f);
  }

  vl53Ready = setupVL53L0X();
  if (!vl53Ready) {
    Serial.println("LOI: Khoi tao VL53L0X that bai. Robot van co the do line.");
  }

  Serial.println("San sang. Mode ban dau: LINE_FOLLOW_MODE");
}

void loop() {
  updateGyroHeading();

  if (currentMode == LINE_FOLLOW_MODE) {
    readLineSensors();
    followLine();

    if (allSensorsBlack()) {
      stopMotors();
      Serial.println("Tat ca 8 mat thay mau den -> Chuyen sang MAZE_SOLVE_MODE");
      delay(300);
      resetHeading(0.0f);
      currentMode = MAZE_SOLVE_MODE;
    }
  } else {
    solveMaze();
  }

  if (millis() - lastDebugMs >= DEBUG_INTERVAL_MS) {
    lastDebugMs = millis();
    printModeDebug();
    printLineDebug();
  }
}

// ===================== MOTOR DRV8833 =====================

void setupMotors() {
  pinMode(MOTOR_STBY_PIN, OUTPUT);
  digitalWrite(MOTOR_STBY_PIN, HIGH);

  ledcSetup(PWM_LEFT_IN1_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_LEFT_IN2_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_RIGHT_IN1_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_RIGHT_IN2_CH, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(LEFT_AIN1_PIN, PWM_LEFT_IN1_CH);
  ledcAttachPin(LEFT_AIN2_PIN, PWM_LEFT_IN2_CH);
  ledcAttachPin(RIGHT_BIN1_PIN, PWM_RIGHT_IN1_CH);
  ledcAttachPin(RIGHT_BIN2_PIN, PWM_RIGHT_IN2_CH);

  stopMotors();
}

void setMotorLeft(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    ledcWrite(PWM_LEFT_IN1_CH, speed);
    ledcWrite(PWM_LEFT_IN2_CH, 0);
  } else if (speed < 0) {
    ledcWrite(PWM_LEFT_IN1_CH, 0);
    ledcWrite(PWM_LEFT_IN2_CH, -speed);
  } else {
    ledcWrite(PWM_LEFT_IN1_CH, 0);
    ledcWrite(PWM_LEFT_IN2_CH, 0);
  }
}

void setMotorRight(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    ledcWrite(PWM_RIGHT_IN1_CH, speed);
    ledcWrite(PWM_RIGHT_IN2_CH, 0);
  } else if (speed < 0) {
    ledcWrite(PWM_RIGHT_IN1_CH, 0);
    ledcWrite(PWM_RIGHT_IN2_CH, -speed);
  } else {
    ledcWrite(PWM_RIGHT_IN1_CH, 0);
    ledcWrite(PWM_RIGHT_IN2_CH, 0);
  }
}

void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  setMotorLeft(leftSpeed);
  setMotorRight(rightSpeed);
}

void stopMotors() {
  setMotorLeft(0);
  setMotorRight(0);
}

// ===================== VL53L0X =====================

bool setupVL53L0X() {
  pinMode(VL53_LEFT_XSHUT_PIN, OUTPUT);
  pinMode(VL53_FRONT_XSHUT_PIN, OUTPUT);
  pinMode(VL53_RIGHT_XSHUT_PIN, OUTPUT);

  // Tat tat ca cam bien de tranh trung dia chi 0x29
  digitalWrite(VL53_LEFT_XSHUT_PIN, LOW);
  digitalWrite(VL53_FRONT_XSHUT_PIN, LOW);
  digitalWrite(VL53_RIGHT_XSHUT_PIN, LOW);
  delay(20);

  // Bat tung cam bien va gan dia chi rieng
  digitalWrite(VL53_LEFT_XSHUT_PIN, HIGH);
  delay(20);
  if (!vl53Left.begin(VL53_LEFT_ADDR, false, &Wire)) {
    Serial.println("VL53L0X trai loi");
    return false;
  }

  digitalWrite(VL53_FRONT_XSHUT_PIN, HIGH);
  delay(20);
  if (!vl53Front.begin(VL53_FRONT_ADDR, false, &Wire)) {
    Serial.println("VL53L0X truoc loi");
    return false;
  }

  digitalWrite(VL53_RIGHT_XSHUT_PIN, HIGH);
  delay(20);
  if (!vl53Right.begin(VL53_RIGHT_ADDR, false, &Wire)) {
    Serial.println("VL53L0X phai loi");
    return false;
  }

  Serial.println("Da khoi tao 3 VL53L0X voi dia chi rieng.");
  return true;
}

int readDistanceMM(Adafruit_VL53L0X &sensor) {
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);

  if (measure.RangeStatus == 4) {
    return 8190; // ngoai tam do, coi nhu rat xa
  }
  return measure.RangeMilliMeter;
}

void readMazeDistances(int &leftMM, int &frontMM, int &rightMM) {
  leftMM = readDistanceMM(vl53Left);
  frontMM = readDistanceMM(vl53Front);
  rightMM = readDistanceMM(vl53Right);
}

// ===================== MPU6050 / GYRO =====================

bool setupMPU6050() {
  if (!mpu.begin(0x68, &Wire)) {
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  return true;
}

void calibrateGyroZ() {
  const int samples = 500;
  float sum = 0.0f;

  Serial.println("Dang calib gyro Z, giu robot dung yen...");
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(3);
  }

  gyroZOffsetRad = sum / samples;
  Serial.print("Gyro Z offset rad/s: ");
  Serial.println(gyroZOffsetRad, 6);
}

void updateGyroHeading() {
  if (!mpuReady) {
    return;
  }

  unsigned long now = millis();
  if (lastGyroUpdateMs == 0) {
    lastGyroUpdateMs = now;
    return;
  }

  float dt = (now - lastGyroUpdateMs) / 1000.0f;
  lastGyroUpdateMs = now;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float gyroZDegPerSec = (g.gyro.z - gyroZOffsetRad) * 57.2957795f;
  currentHeadingDeg += gyroZDegPerSec * dt;
}

void resetHeading(float headingDeg) {
  currentHeadingDeg = headingDeg;
  targetHeadingDeg = headingDeg;
  lastGyroUpdateMs = millis();
}

// ===================== LINE SENSOR / PID-PD =====================

void readLineSensors() {
  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    lineValues[i] = analogRead(linePins[i]);
    lineIsBlack[i] = lineValues[i] < lineThresholds[i];
  }
}

float calculateLineError() {
  // Trong so tu trai sang phai: -3500 .. +3500
  const int weights[NUM_LINE_SENSORS] = {
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
  };

  long weightedSum = 0;
  long activeSum = 0;

  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    int blackStrength = lineThresholds[i] - lineValues[i];
    if (blackStrength > 0) {
      weightedSum += (long)weights[i] * blackStrength;
      activeSum += blackStrength;
    }
  }

  if (activeSum == 0) {
    return lastLineError; // mat line thi giu huong sua loi gan nhat
  }

  float error = (float)weightedSum / (float)activeSum;
  lastLineError = error;
  return error;
}

bool allSensorsBlack() {
  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    if (!lineIsBlack[i]) {
      return false;
    }
  }
  return true;
}

void followLine() {
  float previousError = lastLineError;
  float error = calculateLineError();
  float derivative = error - previousError;

  // Neu line o gan giua, dung gyro de giu huong thang hon.
  float gyroCorrection = 0.0f;
  if (abs(error) < 350.0f) {
    gyroCorrection = (currentHeadingDeg - targetHeadingDeg) * straightGyroKp;
  } else {
    targetHeadingDeg = currentHeadingDeg;
  }

  float correction = lineKp * error + lineKd * derivative + gyroCorrection;

  int leftSpeed = baseSpeedLine + (int)correction;
  int rightSpeed = baseSpeedLine - (int)correction;

  setMotors(constrain(leftSpeed, -255, 255), constrain(rightSpeed, -255, 255));
}

// ===================== MAZE SOLVER =====================

void solveMaze() {
  if (!vl53Ready) {
    stopMotors();
    Serial.println("Maze: chua co VL53L0X, dung robot.");
    delay(300);
    return;
  }

  int leftMM, frontMM, rightMM;
  readMazeDistances(leftMM, frontMM, rightMM);

  bool leftClear = leftMM > WALL_DISTANCE_MM;
  bool frontClear = frontMM > FRONT_CLEAR_DISTANCE_MM;
  bool rightClear = rightMM > WALL_DISTANCE_MM;

  Serial.print("VL53 L/F/R mm: ");
  Serial.print(leftMM);
  Serial.print(" / ");
  Serial.print(frontMM);
  Serial.print(" / ");
  Serial.println(rightMM);

  if (leftClear) {
    Serial.println("Maze: trai trong -> re trai");
    turnLeft90();
    goForwardMaze();
  } else if (frontClear) {
    Serial.println("Maze: truoc trong -> di thang");
    goForwardMaze();
  } else if (rightClear) {
    Serial.println("Maze: phai trong -> re phai");
    turnRight90();
    goForwardMaze();
  } else {
    Serial.println("Maze: ca 3 huong bi chan -> quay dau");
    turnAround180();
  }
}

void goForwardMaze() {
  resetHeading(0.0f);
  unsigned long start = millis();

  while (millis() - start < MAZE_FORWARD_MS) {
    updateGyroHeading();
    int frontMM = readDistanceMM(vl53Front);
    if (frontMM < 80) {
      break;
    }

    float correction = currentHeadingDeg * straightGyroKp;
    int leftSpeed = baseSpeedMaze - (int)correction;
    int rightSpeed = baseSpeedMaze + (int)correction;
    setMotors(constrain(leftSpeed, -255, 255), constrain(rightSpeed, -255, 255));
    delay(5);
  }

  stopMotors();
  delay(80);
}

void turnLeft90() {
  turnByDegrees(-90.0f);
}

void turnRight90() {
  turnByDegrees(90.0f);
}

void turnAround180() {
  turnByDegrees(180.0f);
}

void turnByDegrees(float degrees) {
  resetHeading(0.0f);
  int direction = degrees > 0 ? 1 : -1;
  float target = abs(degrees);
  unsigned long start = millis();

  while (abs(currentHeadingDeg) < target && millis() - start < 3000) {
    updateGyroHeading();

    float remaining = target - abs(currentHeadingDeg);
    int speed = turnSpeedMaze;
    if (remaining < 25.0f) {
      speed = 80; // giam toc gan goc dich
    }

    // Gia dinh gyro Z duong khi robot quay phai. Neu nguoc, doi dau 2 dong duoi.
    setMotors(direction * speed, -direction * speed);
    delay(5);
  }

  stopMotors();
  delay(120);
  resetHeading(0.0f);
}

// ===================== DEBUG SERIAL =====================

void printLineDebug() {
  Serial.print("Line analog: ");
  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    Serial.print(lineValues[i]);
    if (i < NUM_LINE_SENSORS - 1) Serial.print(", ");
  }

  Serial.print(" | BW: ");
  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    Serial.print(lineIsBlack[i] ? "D" : "T"); // D = den, T = trang
    if (i < NUM_LINE_SENSORS - 1) Serial.print(" ");
  }

  Serial.print(" | lineError: ");
  Serial.print(lastLineError);
  Serial.print(" | heading: ");
  Serial.println(currentHeadingDeg);
}

void printModeDebug() {
  Serial.print("Mode: ");
  Serial.println(currentMode == LINE_FOLLOW_MODE ? "LINE_FOLLOW_MODE" : "MAZE_SOLVE_MODE");
}
