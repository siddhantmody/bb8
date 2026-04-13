#include <WiFi.h>
#include <WebServer.h>
#include "BB8WEBSITE.h"
#include <math.h>
#include <ESP32Servo.h>

const char* SSID     = "TP-Link_8A8C";
const char* PASSWORD = "12488674";

const int L_EN  = 17;
const int L_IN1 = 16;
const int L_IN2 = 15;
const int R_EN  = 5;
const int R_IN3 = 7;
const int R_IN4 = 6;

const int SERVO_PIN = 18;
Servo headServo;
int g_servoPos = 90;

const int L_ENC_A = 36;
const int R_ENC_A = 40;

volatile long L_ticks = 0;
volatile long R_ticks = 0;

void IRAM_ATTR isrL() { L_ticks++; }
void IRAM_ATTR isrR() { R_ticks++; }

const int PWM_FREQ = 16000;
const int PWM_BITS = 10;
const int PWM_MAX  = (1 << PWM_BITS) - 1;

inline int pctToDuty(int p) {
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  return (p * PWM_MAX) / 100;
}

WebServer server(80);

enum DriveMode { DRIVE_STOP = 0, DRIVE_FWD = 1, DRIVE_BACK = 2 };
enum SpinMode  { SPIN_STOP  = 0, SPIN_LEFT = 1, SPIN_RIGHT = 2 };

volatile DriveMode g_driveMode = DRIVE_STOP;
volatile SpinMode  g_spinMode  = SPIN_STOP;
volatile int g_base = 60;

unsigned long lastCmdMs = 0;
const unsigned long WD_MS = 5000;

const float TICKS_PER_REV_L = 240.0f;
const float TICKS_PER_REV_R = 240.0f;
const uint32_t CTRL_DT_MS = 50;

float MAX_RPM_DRIVE = 250.0f;
float MAX_RPM_SPIN  = 250.0f;

#define KP 0.3f
#define KI 0.1f
#define KD 0.0f

struct PIDState {
  float integral = 0.0f;
  float prevMeas = 0.0f;
  bool first = true;
};

PIDState pidDrive, pidSpin;

void resetPIDState(PIDState &st) {
  st.integral = 0.0f;
  st.prevMeas = 0.0f;
  st.first = true;
}

int pidcontrol(PIDState &st, float desired_rpm, float measured_rpm, float dt) {
  if (dt <= 0.0f) dt = CTRL_DT_MS / 1000.0f;

  float err = desired_rpm - measured_rpm;

  st.integral += err * dt;
  const float I_MAX = 500.0f;
  if (st.integral > I_MAX) st.integral = I_MAX;
  if (st.integral < -I_MAX) st.integral = -I_MAX;

  float dMeas = 0.0f;
  if (!st.first) dMeas = (measured_rpm - st.prevMeas) / dt;
  else st.first = false;

  st.prevMeas = measured_rpm;

  float u = KP * err + KI * st.integral - KD * dMeas;

  if (u > 100.0f) u = 100.0f;
  if (u < 0.0f) u = 0.0f;

  return (int)roundf(u);
}

void setDriveMotor(int duty, bool forward) {
  if (duty <= 0) {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    ledcWrite(L_EN, 0);
    return;
  }

  if (forward) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
  }

  ledcWrite(L_EN, pctToDuty(duty));
}

void setSpinMotor(int duty, bool forward) {
  if (duty <= 0) {
    digitalWrite(R_IN3, LOW);
    digitalWrite(R_IN4, LOW);
    ledcWrite(R_EN, 0);
    return;
  }

  if (forward) {
    digitalWrite(R_IN3, HIGH);
    digitalWrite(R_IN4, LOW);
  } else {
    digitalWrite(R_IN3, LOW);
    digitalWrite(R_IN4, HIGH);
  }

  ledcWrite(R_EN, pctToDuty(duty));
}

void stopMotors() {
  setDriveMotor(0, true);
  setSpinMotor(0, true);
}

void handleServo() {
  if (server.hasArg("pos")) {
    g_servoPos = constrain(server.arg("pos").toInt(), 0, 180);
    headServo.write(g_servoPos);
  }
  server.send(200, "text/plain", "OK");
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleBase() {
  if (server.hasArg("b")) {
    g_base = constrain(server.arg("b").toInt(), 0, 100);
  }
  lastCmdMs = millis();
  server.send(200, "text/plain", "OK");
}

void handleCmd() {
  if (server.hasArg("b")) {
    g_base = constrain(server.arg("b").toInt(), 0, 100);
  }

  String c = server.hasArg("c") ? server.arg("c") : "";

  if (c == "fwd") {
    g_driveMode = DRIVE_FWD;
  } else if (c == "back") {
    g_driveMode = DRIVE_BACK;
  } else if (c == "drive_stop") {
    g_driveMode = DRIVE_STOP;
  } else if (c == "left") {
    g_spinMode = SPIN_LEFT;
  } else if (c == "right") {
    g_spinMode = SPIN_RIGHT;
  } else if (c == "spin_stop") {
    g_spinMode = SPIN_STOP;
  } else if (c == "stop") {
    g_driveMode = DRIVE_STOP;
    g_spinMode = SPIN_STOP;
  }

  lastCmdMs = millis();
  server.send(200, "text/plain", "OK");
}

float lastDriveRpm = 0.0f;
float lastSpinRpm = 0.0f;

void handleStat() {
  server.sendHeader("Cache-Control", "no-store");
  String out = String(lastDriveRpm, 1) + "," + String(lastSpinRpm, 1);
  server.send(200, "text/plain", out);
}

void setup() {
  Serial.begin(115200);

  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN3, OUTPUT);
  pinMode(R_IN4, OUTPUT);

  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, LOW);

  ledcAttach(L_EN, PWM_FREQ, PWM_BITS);
  ledcAttach(R_EN, PWM_FREQ, PWM_BITS);
  ledcWrite(L_EN, 0);
  ledcWrite(R_EN, 0);

  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), isrR, CHANGE);

  headServo.setPeriodHertz(50);
  headServo.attach(SERVO_PIN, 500, 2400);
  headServo.write(g_servoPos);

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);

  Serial.print("Connecting");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(300);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nIP: http://%s\n", WiFi.localIP().toString().c_str());
    delay(1000);
  } else {
    Serial.println("\nWiFi connect failed.");
  }

  server.on("/", handleRoot);
  server.on("/base", handleBase);
  server.on("/cmd", handleCmd);
  server.on("/servo", handleServo);
  server.on("/stat", handleStat);
  server.begin();

  lastCmdMs = millis();
}

void loop() {
  server.handleClient();

  static unsigned long last = millis();
  unsigned long now = millis();

  if (millis() - lastCmdMs > WD_MS) {
    g_driveMode = DRIVE_STOP;
    g_spinMode = SPIN_STOP;
  }

  if (now - last >= CTRL_DT_MS) {
    float dt = (now - last) / 1000.0f;
    last = now;

    noInterrupts();
    long Lt = L_ticks;
    long Rt = R_ticks;
    L_ticks = 0;
    R_ticks = 0;
    interrupts();

    float drive_rpm = ((Lt / dt) / TICKS_PER_REV_L) * 60.0f;
    float spin_rpm  = ((Rt / dt) / TICKS_PER_REV_R) * 60.0f;

    lastDriveRpm = drive_rpm;
    lastSpinRpm = spin_rpm;

    int base = g_base;
    float baseDrive = MAX_RPM_DRIVE * (base / 100.0f);
    float baseSpin  = MAX_RPM_SPIN  * (base / 100.0f);

    float drive_des = 0.0f;
    float spin_des  = 0.0f;

    switch (g_driveMode) {
      case DRIVE_STOP:
        drive_des = 0.0f;
        break;
      case DRIVE_FWD:
        drive_des = baseDrive;
        break;
      case DRIVE_BACK:
        drive_des = -baseDrive;
        break;
    }

    switch (g_spinMode) {
      case SPIN_STOP:
        spin_des = 0.0f;
        break;
      case SPIN_LEFT:
        spin_des = baseSpin;
        break;
      case SPIN_RIGHT:
        spin_des = -baseSpin;
        break;
    }

    bool drive_fwd = (drive_des >= 0.0f);
    bool spin_fwd  = (spin_des >= 0.0f);

    float drive_des_mag = fabs(drive_des);
    float spin_des_mag  = fabs(spin_des);

    if (drive_des_mag < 1.0f) resetPIDState(pidDrive);
    if (spin_des_mag < 1.0f) resetPIDState(pidSpin);

    int uDrive = (drive_des_mag < 1.0f) ? 0 : pidcontrol(pidDrive, drive_des_mag, fabs(drive_rpm), dt);
    int uSpin  = (spin_des_mag  < 1.0f) ? 0 : pidcontrol(pidSpin,  spin_des_mag,  fabs(spin_rpm),  dt);

    setDriveMotor(uDrive, drive_fwd);
    setSpinMotor(uSpin, spin_fwd);
  }
}