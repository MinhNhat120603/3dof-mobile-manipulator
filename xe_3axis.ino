#define BLYNK_TEMPLATE_ID "your template ID here"
#define BLYNK_TEMPLATE_NAME "your template name here"
#define BLYNK_AUTH_TOKEN "your auth token here"

#include <WiFi.h>
#include <Wire.h>
#include <BlynkSimpleEsp32.h>
// #include <Adafruit_PWMServoDriver.h>  // TODO: Install Adafruit PWM Servo Driver library

char ssid[] = "your WiFi SSID here";
char pass[] = "your WiFi password here";

BlynkTimer timer;

// ======================================================
// PCA9685 SERVO
// ======================================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

#define SERVO_BASE      0
#define SERVO_SHOULDER  1
#define SERVO_ELBOW     2
#define SERVO_GRIPPER   3

float baseAngle = 140;
float shoulderAngle = 90;
float elbowAngle = 0;
float gripperAngle = 0;

float baseTarget = 90;
float shoulderTarget = 60;
float elbowTarget = 90;
float gripperTarget = 50;

float SERVO_STEP = 2.5;

#define BASE_MIN      0
#define BASE_MAX      180

#define SHOULDER_MIN  0
#define SHOULDER_MAX  180

#define ELBOW_MIN     0
#define ELBOW_MAX     180

#define GRIPPER_MIN   0
#define GRIPPER_MAX   180

// ======================================================
// DRV8833 MOTOR
// ======================================================
#define FL_IN1 25
#define FL_IN2 26

#define FR_IN1 27
#define FR_IN2 14

#define RL_IN1 32
#define RL_IN2 33

#define RR_IN1 18
#define RR_IN2 19

#define MAX_SPEED 110

#define MOTOR_RAMP_STEP 12

int SIGN_FL = 1;
int SIGN_FR = 1;
int SIGN_RL = 1;
int SIGN_RR = 1;

int curFL = 0;
int curFR = 0;
int curRL = 0;
int curRR = 0;

int tarFL = 0;
int tarFR = 0;
int tarRL = 0;
int tarRR = 0;

// ======================================================
// BLYNK VARIABLES
// ======================================================
bool autoMode = false;

int moveFB = 0;      // V1
int moveLR = 0;      // V2
int rotateZ = 0;     // V3

bool autoPickRequest = false;
bool autoRunning = false;
int autoStep = 0;
unsigned long autoStepStart = 0;

// ======================================================
// SERVO FUNCTIONS
// ======================================================
int usToPwm(float us) {
  float period_us = 1000000.0 / SERVO_FREQ;
  return (int)(us * 4096.0 / period_us);
}

int angleToPwm(float angle) {
  angle = constrain(angle, 0, 180);
  float us = SERVO_MIN_US + (angle / 180.0) * (SERVO_MAX_US - SERVO_MIN_US);
  return usToPwm(us);
}

void writeServo(int channel, float angle) {
  pwm.setPWM(channel, 0, angleToPwm(angle));
}

float approachFloat(float current, float target, float step) {
  if (abs(target - current) <= step) return target;

  if (target > current) return current + step;
  else return current - step;
}

void setArmTarget(float base, float shoulder, float elbow, float gripper) {
  baseTarget = constrain(base, BASE_MIN, BASE_MAX);
  shoulderTarget = constrain(shoulder, SHOULDER_MIN, SHOULDER_MAX);
  elbowTarget = constrain(elbow, ELBOW_MIN, ELBOW_MAX);
  gripperTarget = constrain(gripper, GRIPPER_MIN, GRIPPER_MAX);

  Blynk.virtualWrite(V10, (int)baseTarget);
  Blynk.virtualWrite(V11, (int)shoulderTarget);
  Blynk.virtualWrite(V12, (int)elbowTarget);
  Blynk.virtualWrite(V13, (int)gripperTarget);
}

void servoSmoothTask() {
  baseAngle = approachFloat(baseAngle, baseTarget, SERVO_STEP);
  shoulderAngle = approachFloat(shoulderAngle, shoulderTarget, SERVO_STEP);
  elbowAngle = approachFloat(elbowAngle, elbowTarget, SERVO_STEP);
  gripperAngle = approachFloat(gripperAngle, gripperTarget, SERVO_STEP);

  writeServo(SERVO_BASE, baseAngle);
  writeServo(SERVO_SHOULDER, shoulderAngle);
  writeServo(SERVO_ELBOW, elbowAngle);
  writeServo(SERVO_GRIPPER, gripperAngle);
}

void armHome() {
  setArmTarget(90, 60, 90, 50);
}

void gripperOpen() {
  setArmTarget(baseTarget, shoulderTarget, elbowTarget, 35);
}

void gripperClose() {
  setArmTarget(baseTarget, shoulderTarget, elbowTarget, 90);
}

// ======================================================
// MOTOR FUNCTIONS
// ======================================================
void setupMotorPins() {
  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT);
  pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT);
  pinMode(RR_IN2, OUTPUT);
}

void setMotorRaw(int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    analogWrite(in1, speed);
    analogWrite(in2, 0);
  } 
  else if (speed < 0) {
    analogWrite(in1, 0);
    analogWrite(in2, -speed);
  } 
  else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

int approachInt(int current, int target, int step) {
  if (abs(target - current) <= step) return target;

  if (target > current) return current + step;
  else return current - step;
}

void setWheelTargets(int fl, int fr, int rl, int rr) {
  tarFL = constrain(fl * SIGN_FL, -MAX_SPEED, MAX_SPEED);
  tarFR = constrain(fr * SIGN_FR, -MAX_SPEED, MAX_SPEED);
  tarRL = constrain(rl * SIGN_RL, -MAX_SPEED, MAX_SPEED);
  tarRR = constrain(rr * SIGN_RR, -MAX_SPEED, MAX_SPEED);
}

void motorRampTask() {
  curFL = approachInt(curFL, tarFL, MOTOR_RAMP_STEP);
  curFR = approachInt(curFR, tarFR, MOTOR_RAMP_STEP);
  curRL = approachInt(curRL, tarRL, MOTOR_RAMP_STEP);
  curRR = approachInt(curRR, tarRR, MOTOR_RAMP_STEP);

  setMotorRaw(FL_IN1, FL_IN2, curFL);
  setMotorRaw(FR_IN1, FR_IN2, curFR);
  setMotorRaw(RL_IN1, RL_IN2, curRL);
  setMotorRaw(RR_IN1, RR_IN2, curRR);
}

void stopRobot() {
  setWheelTargets(0, 0, 0, 0);
}

// ======================================================
// MECANUM DRIVE
// vx: forward/backward
// vy: left/right
// wz: rotation
// ======================================================
void mecanumDrive(int vx, int vy, int wz) 
{
  // Công thức mecanum phổ biến
  int fl = vx - vy - wz;
  int fr = vx + vy + wz;
  int rl = vx + vy - wz;
  int rr = vx - vy + wz;

  int maxVal = max(max(abs(fl), abs(fr)), max(abs(rl), abs(rr)));

  if (maxVal > MAX_SPEED) {
    fl = fl * MAX_SPEED / maxVal;
    fr = fr * MAX_SPEED / maxVal;
    rl = rl * MAX_SPEED / maxVal;
    rr = rr * MAX_SPEED / maxVal;
  }

  setWheelTargets(fl, fr, rl, rr);
}

// ======================================================
// BLYNK CONNECT
// ======================================================
BLYNK_CONNECTED() {
  Blynk.virtualWrite(V20, "ESP32 Connected");

  Blynk.virtualWrite(V0, autoMode);
  Blynk.virtualWrite(V1, 0);
  Blynk.virtualWrite(V2, 0);
  Blynk.virtualWrite(V3, 0);

  Blynk.virtualWrite(V10, (int)baseTarget);
  Blynk.virtualWrite(V11, (int)shoulderTarget);
  Blynk.virtualWrite(V12, (int)elbowTarget);
  Blynk.virtualWrite(V13, (int)gripperTarget);
}

// ======================================================
// BLYNK HANDLERS
// ======================================================
BLYNK_WRITE(V0) {
  autoMode = param.asInt();

  if (autoMode) {
    stopRobot();
    moveFB = 0;
    moveLR = 0;
    rotateZ = 0;

    Blynk.virtualWrite(V1, 0);
    Blynk.virtualWrite(V2, 0);
    Blynk.virtualWrite(V3, 0);
    Blynk.virtualWrite(V20, "AUTO MODE");
  } 
  else {
    autoRunning = false;
    autoPickRequest = false;
    stopRobot();

    Blynk.virtualWrite(V20, "MANUAL MODE");
  }
}

BLYNK_WRITE(V1) {
  moveFB = param.asInt();
}

BLYNK_WRITE(V2) {
  rotateZ = param.asInt();
}

BLYNK_WRITE(V3) {
  moveLR = param.asInt();
}

BLYNK_WRITE(V4) {
  int value = param.asInt();

  if (value == 1) {
    autoRunning = false;
    autoPickRequest = false;

    moveFB = 0;
    moveLR = 0;
    rotateZ = 0;

    stopRobot();

    Blynk.virtualWrite(V1, 0);
    Blynk.virtualWrite(V2, 0);
    Blynk.virtualWrite(V3, 0);
    Blynk.virtualWrite(V20, "STOP");
  }
}

BLYNK_WRITE(V5) {
  int value = param.asInt();

  if (value == 1) {
    autoMode = true;
    autoPickRequest = true;

    Blynk.virtualWrite(V0, 1);
    Blynk.virtualWrite(V20, "AUTO PICK REQUEST");
  }
}

BLYNK_WRITE(V10) {
  if (!autoMode) {
    baseTarget = constrain(param.asInt(), BASE_MIN, BASE_MAX);
  }
}

BLYNK_WRITE(V11) {
  if (!autoMode) {
    shoulderTarget = constrain(param.asInt(), SHOULDER_MIN, SHOULDER_MAX);
  }
}

BLYNK_WRITE(V12) {
  if (!autoMode) {
    elbowTarget = constrain(param.asInt(), ELBOW_MIN, ELBOW_MAX);
  }
}

BLYNK_WRITE(V13) {
  if (!autoMode) {
    gripperTarget = constrain(param.asInt(), GRIPPER_MIN, GRIPPER_MAX);
  }
}

// ======================================================
// MANUAL CONTROL
// ======================================================
void manualControlTask() {
  if (autoMode) return;

  int vx = map(moveFB, -100, 100, -MAX_SPEED, MAX_SPEED);

  int vy = map(moveLR, -100, 100, MAX_SPEED, -MAX_SPEED);

  int wz = map(rotateZ, -100, 100, MAX_SPEED, -MAX_SPEED);

  if (abs(vx) < 10) vx = 0;
  if (abs(vy) < 10) vy = 0;
  if (abs(wz) < 10) wz = 0;

  mecanumDrive(vx, vy, wz);
}

// ======================================================
// AUTO PICK STATE MACHINE
// ======================================================
void startAutoPick() {
  autoRunning = true;
  autoStep = 0;
  autoStepStart = millis();
  Blynk.virtualWrite(V20, "Auto: Start");
}

void nextAutoStep() {
  autoStep++;
  autoStepStart = millis();
}

bool stepTimePassed(unsigned long durationMs) {
  return millis() - autoStepStart >= durationMs;
}

void autoPickTask() {
  if (!autoMode) return;

  if (autoPickRequest && !autoRunning) {
    autoPickRequest = false;
    startAutoPick();
  }

  if (!autoRunning) return;

  switch (autoStep) {
    case 0:
      Blynk.virtualWrite(V20, "Auto: Home");
      armHome();
      nextAutoStep();
      break;

    case 1:
      if (stepTimePassed(1200)) {
        Blynk.virtualWrite(V20, "Auto: Forward");
        mecanumDrive(70, 0, 0);
        nextAutoStep();
      }
      break;

    case 2:
      if (stepTimePassed(1400)) {
        stopRobot();
        Blynk.virtualWrite(V20, "Auto: Lower Arm");

        // Hạ tay từ từ
        setArmTarget(90, 95, 65, 35);

        nextAutoStep();
      }
      break;

    case 3:
      if (stepTimePassed(1600)) {
        Blynk.virtualWrite(V20, "Auto: Grip");
        gripperClose();
        nextAutoStep();
      }
      break;

    case 4:
      if (stepTimePassed(1000)) {
        Blynk.virtualWrite(V20, "Auto: Lift");

        setArmTarget(90, 60, 90, 90);

        nextAutoStep();
      }
      break;

    case 5:
      if (stepTimePassed(1400)) {
        Blynk.virtualWrite(V20, "Auto: Backward");
        mecanumDrive(-70, 0, 0);
        nextAutoStep();
      }
      break;

    case 6:
      if (stepTimePassed(1200)) {
        stopRobot();
        Blynk.virtualWrite(V20, "Auto: Rotate Arm");

        setArmTarget(135, 60, 90, 90);

        nextAutoStep();
      }
      break;

    case 7:
      if (stepTimePassed(1500)) {
        Blynk.virtualWrite(V20, "Auto: Release");
        gripperOpen();
        nextAutoStep();
      }
      break;

    case 8:
      if (stepTimePassed(1000)) {
        Blynk.virtualWrite(V20, "Auto: Done");

        armHome();
        stopRobot();

        autoRunning = false;
        autoMode = false;

        Blynk.virtualWrite(V0, 0);
        Blynk.virtualWrite(V5, 0);
        Blynk.virtualWrite(V20, "MANUAL READY");
      }
      break;
  }
}

// ======================================================
// STATUS
// ======================================================
void statusTask() {
  if (autoRunning) return;

  if (autoMode) {
    Blynk.virtualWrite(V20, "AUTO READY");
  } 
  else {
    Blynk.virtualWrite(V20, "MANUAL READY");
  }
}

// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("ESP32 Blynk Mobile Manipulator Smooth Control");

  Wire.begin(21, 22);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(500);

  setupMotorPins();
  stopRobot();

  armHome();

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  timer.setInterval(20L, servoSmoothTask);
  timer.setInterval(20L, motorRampTask);
  timer.setInterval(50L, manualControlTask);
  timer.setInterval(50L, autoPickTask);
  timer.setInterval(1000L, statusTask);

  Blynk.virtualWrite(V20, "Robot Ready");
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  Blynk.run();
  timer.run();
}