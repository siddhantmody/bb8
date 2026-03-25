#include <WiFi.h>
#include <WebServer.h>
#include "pageFinal.h"
#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include "vive510.h"     // Vive tracker library (vive510.h / vive510.cpp in same folder)
#include <math.h>

// ===================== WiFi config =====================
const char* SSID     = "TP-Link_8A8C";
const char* PASSWORD = "12488674";

// ===================== L298N motor driver pins =====================
const int L_EN  = 17;   // L_ENA
const int L_IN1 = 16;   // L_IN1
const int L_IN2 = 15;   // L_IN2
const int R_EN  = 5;    // R_ENA
const int R_IN3 = 7;    // R_IN3
const int R_IN4 = 6;    // R_IN4

// Right wheel inversion switch
const bool RIGHT_DIR_INVERT = false;
// ===================== TopHat I2C configuration =====================
#define TOPHAT_ADDR   0x28
#define TOPHAT_SDA    12
#define TOPHAT_SCL    13

volatile uint16_t wifi_packets = 0;   // WiFi packets in the last 0.5 s
uint32_t last_i2c_time = 0;
const uint32_t I2C_PERIOD_MS = 500;   // send to TopHat at 2 Hz
uint8_t current_health = 255;         // health returned from TopHat
// ===================== Encoder pins (A channel only) =====================
const int L_ENC_A = 36;
const int R_ENC_A = 40;

// per-window + cumulative ticks
volatile long L_ticks = 0, R_ticks = 0;
volatile long L_dead  = 0, R_dead  = 0;

void IRAM_ATTR isrL(){ L_ticks++; L_dead++; }
void IRAM_ATTR isrR(){ R_ticks++; R_dead++; }

// ===================== TopHat I2C function =====================
void sendTopHatUpdate()
{
  uint8_t packets_to_send = (wifi_packets > 255) ? 255 : (uint8_t)wifi_packets;

  Wire.beginTransmission(TOPHAT_ADDR);
  Wire.write(packets_to_send);
  uint8_t err = Wire.endTransmission();

  if (err != 0) {
    Serial.printf("[I2C] Write error: %d\n", err);
    return;
  }

  uint8_t n = Wire.requestFrom(TOPHAT_ADDR, (uint8_t)1);
  if (n == 1 && Wire.available()) {
    current_health = Wire.read();
  } else {
    Serial.println("[I2C] Read failed");
  }
}


// ===================== PWM settings =====================
const int PWM_FREQ = 16000;
const int PWM_BITS = 10;
const int PWM_MAX  = (1<<PWM_BITS)-1;

const int SERVO_PIN       = 3;
const int SERVO_BITS      = 12;
const int SERVO_FREQ      = 50;     // 20 ms period
const int SERVO_MAX_DUTY  = (1 << SERVO_BITS) - 1;

// Map 0–100% to duty range
inline int pctToDuty(int p){
  if(p < 0)   p = 0;
  if(p > 100) p = 100;
  return (p * PWM_MAX) / 100;
}

// ===================== Drive state and watchdog =====================
WebServer server(80);
enum Mode{STOP=0,FWD=1,BACK=2,LEFT=3,RIGHT=4};
volatile Mode g_mode = STOP;
volatile int  g_base = 60;
unsigned long lastCmdMs = 0;
const unsigned long WD_MS = 5000;

// ===================== Control timing / encoder =====================
const float    TICKS_PER_REV_L = 240.0f;
const float    TICKS_PER_REV_R = 240.0f;
const uint32_t CTRL_DT_MS      = 50;

// Max RPM
float MAX_RPM_L = 500.0f;
float MAX_RPM_R = 500.0f;

// ===================== PID gains =====================
#define KP  0.3f
#define KI  0.1f
#define KD  0.0f   // PI control

// Per-wheel PID state
struct PIDState {
  float integral = 0.0f;
  float prevMeas = 0.0f;
  bool  first    = true;
};

// Forward declaration
int pidcontrol(PIDState &st, float desired_rpm, float measured_rpm, float dt);

// Helper: reset PID (used by dead-reckon segments)
static inline void resetPIDState(PIDState &st){
  st.integral = 0.0f;
  st.prevMeas = 0.0f;
  st.first    = true;
}

PIDState pidL, pidR;

// ===================== TOF / Wall-follow =====================
#define SDA_PIN  41
#define SCL_PIN  2

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

#define SHT_LOX1 21
#define SHT_LOX2 33

TwoWire WireToF = TwoWire(1);

Adafruit_VL53L0X lox1;
Adafruit_VL53L0X lox2;

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

const int FRONT_TURN_DIST = 200;

volatile bool g_wallFollow = false;

// ===================== Vive configuration (UNCOMMENTED in your main) =====================
// #define SIGNALPIN1  35
// #define SIGNALPIN2  9

// Vive510 vive1(SIGNALPIN1);
// Vive510 vive2(SIGNALPIN2);

// float robotX     = 0.0f;
// float robotY     = 0.0f;
// float robotTheta = 0.0f;   // radians [-pi, pi]

// int16_t vive1_x = 0, vive1_y = 0;
// int16_t vive2_x = 0, vive2_y = 0;

// // Vive helper
// bool updateVive(Vive510 &vive, int16_t &x, int16_t &y, uint8_t syncTries)
// {
//   if (vive.status() == VIVE_RECEIVING) {
//     x = vive.xCoord();
//     y = vive.yCoord();
//     return true;
//   } else {
//     vive.sync(syncTries);
//     return false;
//   }
// }

// ===================== Dead-reckon mode flags =====================
volatile bool g_dr1     = false;
volatile bool g_dr2     = false;
volatile bool g_viveDR  = false;

// Convenience: cancel all autonomous modes
static inline void cancelAutoModes(){
  g_wallFollow = false;
  g_dr1 = false;
  g_dr2 = false;
  g_viveDR = false;
}

// ===================== TOF init helper =====================
void setID() {
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  if (!lox1.begin(LOX1_ADDRESS, false, &WireToF)) {
    Serial.println(F("Failed to boot first VL53L0X (LEFT)"));
    while (1);
  }
  delay(10);

  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  if (!lox2.begin(LOX2_ADDRESS, false, &WireToF)) {
    Serial.println(F("Failed to boot second VL53L0X (FRONT)"));
    while (1);
  }
}

uint32_t degreesToDuty(int angle) {
  if (angle < 0)   angle = 0;
  if (angle > 180) angle = 180;

  const float minDuty = 0.025f;
  const float maxDuty = 0.125f;

  float percentage = minDuty + ((angle / 180.0f) * (maxDuty - minDuty));
  return (uint32_t)(percentage * SERVO_MAX_DUTY);
}

// ===================== Motor output functions =====================
void setLeft(int duty, bool forward){
  if(duty <= 0){
    digitalWrite(L_IN1,LOW);
    digitalWrite(L_IN2,LOW);
    ledcWrite(L_EN,0);
    return;
  }
  if(forward){
    digitalWrite(L_IN1,HIGH);
    digitalWrite(L_IN2,LOW);
  }else{
    digitalWrite(L_IN1,LOW);
    digitalWrite(L_IN2,HIGH);
  }
  ledcWrite(L_EN, pctToDuty(duty));
}

void setRight(int duty, bool forward){
  if(duty <= 0){
    digitalWrite(R_IN3,LOW);
    digitalWrite(R_IN4,LOW);
    ledcWrite(R_EN,0);
    return;
  }
  bool f = RIGHT_DIR_INVERT ? !forward : forward;
  if(f){
    digitalWrite(R_IN3,HIGH);
    digitalWrite(R_IN4,LOW);
  }else{
    digitalWrite(R_IN3,LOW);
    digitalWrite(R_IN4,HIGH);
  }
  ledcWrite(R_EN, pctToDuty(duty));
}

// ===================== Attack arm =====================
volatile int swing = 1;
void AttackArm(){
  uint32_t duty0   = degreesToDuty(0);
  uint32_t duty135 = degreesToDuty(135);

  if(swing==1){
    ledcWrite(SERVO_PIN, duty135);
    swing = 0;
    Serial.println("swing out");
  } else {
    ledcWrite(SERVO_PIN, duty0);
    swing = 1;
    Serial.println("swing back");
  }
}

// ===================== Web Handlers =====================
void handleRoot(){
  server.send_P(200,"text/html",INDEX_HTML);
}

void handleBase(){
  if(server.hasArg("b"))
    g_base = constrain(server.arg("b").toInt(),0,100);
  lastCmdMs = millis();
  server.send(200,"text/plain","OK");
}

// --- Forward declare start functions for modes
void dr1Start();
void dr2Start();
void viveDRStart();

void handleCmd(){
  if(server.hasArg("b"))
    g_base = constrain(server.arg("b").toInt(),0,100);

  String c = server.hasArg("c") ? server.arg("c") : "";

  // Manual direction commands cancel autonomous modes
  if(c=="fwd"){
    wifi_packets++;
    cancelAutoModes();
    g_mode = FWD;
  }
  else if(c=="back"){
    wifi_packets++;
    cancelAutoModes();
    g_mode = BACK;
  }
  else if(c=="left"){
    wifi_packets++;
    cancelAutoModes();
    g_mode = LEFT;
  }
  else if(c=="right"){
    wifi_packets++;
    cancelAutoModes();
    g_mode = RIGHT;
  }
  else if(c=="stop"){
    wifi_packets++;
    cancelAutoModes();
    g_mode = STOP;
  }
  else if(c=="wall"){
    // wall-follow is its own mode; cancel others
    g_dr1 = g_dr2 = g_viveDR = false;
    g_wallFollow = true;
    g_mode = FWD;
  }
  else if(c=="attack"){
    // do not change driving mode, just swing arm
    wifi_packets++;
    AttackArm();
  }
  // --- Dead-reckon modes ---
  else if(c=="dr1" || c=="nexus"){
    g_wallFollow = false; g_dr2 = false; g_viveDR = false;
    g_dr1 = true;
    dr1Start();
    g_mode = FWD;
  }
  else if(c=="dr2" || c=="mid"){
    g_wallFollow = false; g_dr1 = false; g_viveDR = false;
    g_dr2 = true;
    dr2Start();
    g_mode = FWD;
  }
  else if(c=="vive_dr" || c=="vive"){
    g_wallFollow = false; g_dr1 = false; g_dr2 = false;
    g_viveDR = true;
    viveDRStart();
    g_mode = FWD;
  }
  else{
    g_wallFollow = false;
  }

  lastCmdMs = millis();
  server.send(200,"text/plain","OK");
}

float lastLrpm = 0, lastRrpm = 0;
void handleStat(){
  server.sendHeader("Cache-Control","no-store");
  String out = String(lastLrpm,1) + "," + String(lastRrpm,1);
  server.send(200,"text/plain",out);
}

// ===================== PID implementation =====================
int pidcontrol(PIDState &st, float desired_rpm, float measured_rpm, float dt){
  if (dt <= 0.0f) dt = CTRL_DT_MS / 1000.0f;

  float err = desired_rpm - measured_rpm;

  st.integral += err * dt;
  const float I_MAX = 500.0f;
  if (st.integral >  I_MAX) st.integral =  I_MAX;
  if (st.integral < -I_MAX) st.integral = -I_MAX;

  float dMeas = 0.0f;
  if(!st.first) dMeas = (measured_rpm - st.prevMeas) / dt;
  else st.first = false;

  st.prevMeas = measured_rpm;

  float u = KP * err + KI * st.integral - KD * dMeas;

  if(u > 100.0f) u = 100.0f;
  if(u < 0.0f)   u = 0.0f;

  return (int)roundf(u);
}

// ===================== Wall-follow compute =====================
void wallFollowCompute(float baseL, float baseR, float &L_des, float &R_des) {
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);

  int dSide  = (measure1.RangeStatus != 4) ? measure1.RangeMilliMeter : -1;
  int dFront = (measure2.RangeStatus != 4) ? measure2.RangeMilliMeter : -1;

  float L_scale = 1.0f;
  float R_scale = 1.0f;

  if (dFront > 0 && dFront < FRONT_TURN_DIST) {
    L_scale = 1.0f;
    R_scale = -1.0f;
    Serial.println("FRONT TOO CLOSE -> HARD LEFT TURN (pivot)");
  } else {
    if (dSide <= 0 || dSide > 500) {
      L_scale = 0.0f;
      R_scale = 1.0f;
      Serial.println("BAD SIDE READING -> STRAIGHT USING RIGHT");
    } 
    else if (dSide < 125) {
      L_scale = 1.0f;
      R_scale = 0.5f;
      Serial.println("TOO CLOSE TO WALL -> TURN RIGHT WHILE FORWARD");
    } 
    else {
      L_scale = 0.5f;
      R_scale = 1.0f;
      Serial.println("TOO FAR FROM WALL -> TURN LEFT WHILE FORWARD");
    }
  }

  L_des = baseL * L_scale;
  R_des = baseR * R_scale;
}

// ============================================================================
// DEAD RECKON #1 (DR1)  -- signs preserved exactly as you wrote
// ============================================================================
enum DR1State { DR1_DRIVE1, DR1_TURN1, DR1_DRIVE2, DR1_TURN2, DR1_DRIVE3, DR1_DONE };
static DR1State dr1State = DR1_DRIVE1;
static long dr1SegStartL = 0, dr1SegStartR = 0;

// Tunables
static float dr1_baseRPM_drive = 450.0f;
static float dr1_baseRPM_turn  = 250.0f;

// Tick goals
static long DR1_DRIVE1_TICKS = 10129;
static long DR1_TURN1_TICKS  = 1000;
static long DR1_DRIVE2_TICKS = 862;
static long DR1_TURN2_TICKS  = 1000;
static long DR1_DRIVE3_TICKS = 1940;

static inline long dr1_avgTicks(long dL, long dR){ return (labs(dL) + labs(dR)) / 2; }
static inline long dr1_turnTicks(long dL, long dR){ return (labs(dL) + labs(dR)) / 2; }

static inline void dr1StartSegment(){
  noInterrupts();
  dr1SegStartL = L_dead;
  dr1SegStartR = R_dead;
  interrupts();
  resetPIDState(pidL);
  resetPIDState(pidR);
}

void dr1Start(){
  dr1State = DR1_DRIVE1;
  dr1StartSegment();
}

bool dr1Compute(float &L_des, float &R_des){
  noInterrupts();
  long curL = L_dead;
  long curR = R_dead;
  interrupts();

  long dL = curL - dr1SegStartL;
  long dR = curR - dr1SegStartR;

  switch(dr1State){

    case DR1_DRIVE1:
      if(dr1_avgTicks(dL, dR) >= DR1_DRIVE1_TICKS){
        L_des = 0; R_des = 0;
        dr1State = DR1_TURN1;
        dr1StartSegment();
      } else {
        L_des = dr1_baseRPM_drive;
        R_des = dr1_baseRPM_drive;
      }
      break;

    case DR1_TURN1:
      if(dr1_turnTicks(dL, dR) >= DR1_TURN1_TICKS){
        L_des = 0; R_des = 0;
        dr1State = DR1_DRIVE2;
        dr1StartSegment();
      } else {
        // KEEP SIGNS AS PROVIDED:
        L_des = dr1_baseRPM_turn;
        R_des = -dr1_baseRPM_turn;
      }
      break;

    case DR1_DRIVE2:
      if(dr1_avgTicks(dL, dR) >= DR1_DRIVE2_TICKS){
        L_des = 0; R_des = 0;
        dr1State = DR1_TURN2;
        dr1StartSegment();
      } else {
        L_des = dr1_baseRPM_drive;
        R_des = dr1_baseRPM_drive;
      }
      break;

    case DR1_TURN2:
      if(dr1_turnTicks(dL, dR) >= DR1_TURN2_TICKS){
        L_des = 0; R_des = 0;
        dr1State = DR1_DRIVE3;
        dr1StartSegment();
      } else {
        // KEEP SIGNS AS PROVIDED:
        L_des = -dr1_baseRPM_turn;
        R_des = dr1_baseRPM_turn;
      }
      break;

    case DR1_DRIVE3:
      if(dr1_avgTicks(dL, dR) >= DR1_DRIVE3_TICKS){
        L_des = 0; R_des = 0;
        dr1State = DR1_DONE;
        dr1StartSegment();
      } else {
        L_des = dr1_baseRPM_drive;
        R_des = dr1_baseRPM_drive;
      }
      break;

    case DR1_DONE:
    default:
      L_des = 0; R_des = 0;
      return false;
  }

  return true;
}

// ============================================================================
// DEAD RECKON #2 (DR2) -- signs preserved exactly as you wrote
// ============================================================================
enum DR2State {
  DR2_DRIVE1, DR2_TURN1, DR2_DRIVE2, DR2_TURN2, DR2_DRIVE3,
  DR2_DRIVE4, DR2_DRIVE5, DR2_DRIVE6, DR2_DRIVE7, DR2_DRIVE8,
  DR2_DRIVE9, DR2_DRIVE10, DR2_DRIVE11, DR2_DONE
};
static DR2State dr2State = DR2_DRIVE1;
static long dr2SegStartL = 0, dr2SegStartR = 0;

// Tunables
static float dr2_baseRPM_drive = 450.0f;
static float dr2_baseRPM_turn  = 250.0f;

// Tick goals
static long DR2_DRIVE1_TICKS  = 10129;
static long DR2_TURN1_TICKS   = 1000;
static long DR2_DRIVE2_TICKS  = 862;
static long DR2_TURN2_TICKS   = 950;
static long DR2_DRIVE3_TICKS  = 4310;
static long DR2_DRIVE4_TICKS  = 250;
static long DR2_DRIVE5_TICKS  = 500;
static long DR2_DRIVE6_TICKS  = 250;
static long DR2_DRIVE7_TICKS  = 500;
static long DR2_DRIVE8_TICKS  = 250;
static long DR2_DRIVE9_TICKS  = 500;
static long DR2_DRIVE10_TICKS = 250;
static long DR2_DRIVE11_TICKS = 500;

static inline long dr2_avgTicks(long dL, long dR){ return (labs(dL) + labs(dR)) / 2; }
static inline long dr2_turnTicks(long dL, long dR){ return (labs(dL) + labs(dR)) / 2; }

static inline void dr2StartSegment(){
  noInterrupts();
  dr2SegStartL = L_dead;
  dr2SegStartR = R_dead;
  interrupts();
  resetPIDState(pidL);
  resetPIDState(pidR);
}

void dr2Start(){
  dr2State = DR2_DRIVE1;
  dr2StartSegment();
}

bool dr2Compute(float &L_des, float &R_des){
  noInterrupts();
  long curL = L_dead;
  long curR = R_dead;
  interrupts();

  long dL = curL - dr2SegStartL;
  long dR = curR - dr2SegStartR;

  switch(dr2State){

    case DR2_DRIVE1:
      if(dr2_avgTicks(dL,dR) >= DR2_DRIVE1_TICKS){
        L_des = 0; R_des = 0;
        dr2State = DR2_TURN1;
        dr2StartSegment();
      } else {
        L_des = dr2_baseRPM_drive;
        R_des = dr2_baseRPM_drive;
      }
      break;

    case DR2_TURN1:
      if(dr2_turnTicks(dL,dR) >= DR2_TURN1_TICKS){
        L_des = 0; R_des = 0;
        dr2State = DR2_DRIVE2;
        dr2StartSegment();
      } else {
        // KEEP SIGNS AS PROVIDED:
        L_des = dr2_baseRPM_turn;
        R_des = -dr2_baseRPM_turn;
      }
      break;

    case DR2_DRIVE2:
      if(dr2_avgTicks(dL,dR) >= DR2_DRIVE2_TICKS){
        L_des = 0; R_des = 0;
        dr2State = DR2_TURN2;
        dr2StartSegment();
      } else {
        L_des = dr2_baseRPM_drive;
        R_des = dr2_baseRPM_drive;
      }
      break;

    case DR2_TURN2:
      if(dr2_turnTicks(dL,dR) >= DR2_TURN2_TICKS){
        L_des = 0; R_des = 0;
        dr2State = DR2_DRIVE3;
        dr2StartSegment();
      } else {
        // KEEP SIGNS AS PROVIDED:
        L_des = -dr2_baseRPM_turn;
        R_des = dr2_baseRPM_turn;
      }
      break;

    case DR2_DRIVE3:
      if(dr2_avgTicks(dL,dR) >= DR2_DRIVE3_TICKS){
        L_des = 0; R_des = 0;
        dr2State = DR2_DRIVE4;
        dr2StartSegment();
      } else {
        L_des = dr2_baseRPM_drive;
        R_des = dr2_baseRPM_drive;
      }
      break;

    case DR2_DRIVE4:
      if(dr2_avgTicks(dL,dR) >= DR2_DRIVE4_TICKS){
        L_des = 0; R_des = 0;
        dr2State = DR2_DRIVE5;
        dr2StartSegment();
      } else {
        L_des = -dr2_baseRPM_drive;
        R_des = -dr2_baseRPM_drive;
      }
      break;

    case DR2_DRIVE5:
      if(dr2_avgTicks(dL,dR) >= DR2_DRIVE5_TICKS){
        L_des = 0; R_des = 0;
        dr2State = DR2_DRIVE6;
        dr2StartSegment();
      } else {
        L_des = dr2_baseRPM_drive;
        R_des = dr2_baseRPM_drive;
      }
      break;

    case DR2_DRIVE6:
      if(dr2_avgTicks(dL,dR) >= DR2_DRIVE6_TICKS){
        L_des = 0; R_des = 0;
        dr2State = DR2_DRIVE7;
        dr2StartSegment();
      } else {
        L_des = -dr2_baseRPM_drive;
        R_des = -dr2_baseRPM_drive;
      }
      break;

    case DR2_DRIVE7:
      if(dr2_avgTicks(dL,dR) >= DR2_DRIVE7_TICKS){
        L_des = 0; R_des = 0;
        dr2State = DR2_DRIVE8;
        dr2StartSegment();
      } else {
        L_des = dr2_baseRPM_drive;
        R_des = dr2_baseRPM_drive;
      }
      break;

    case DR2_DRIVE8:
      if(dr2_avgTicks(dL,dR) >= DR2_DRIVE8_TICKS){
        L_des = 0; R_des = 0;
        dr2State = DR2_DRIVE9;
        dr2StartSegment();
      } else {
        L_des = -dr2_baseRPM_drive;
        R_des = -dr2_baseRPM_drive;
      }
      break;

    case DR2_DRIVE9:
      if(dr2_avgTicks(dL,dR) >= DR2_DRIVE9_TICKS){
        L_des = 0; R_des = 0;
        dr2State = DR2_DRIVE10;
        dr2StartSegment();
      } else {
        L_des = dr2_baseRPM_drive;
        R_des = dr2_baseRPM_drive;
      }
      break;

    case DR2_DRIVE10:
      if(dr2_avgTicks(dL,dR) >= DR2_DRIVE10_TICKS){
        L_des = 0; R_des = 0;
        dr2State = DR2_DRIVE11;
        dr2StartSegment();
      } else {
        L_des = dr2_baseRPM_drive;
        R_des = dr2_baseRPM_drive;
      }
      break;

    case DR2_DRIVE11:
      if(dr2_avgTicks(dL,dR) >= DR2_DRIVE11_TICKS){
        L_des = 0; R_des = 0;
        dr2State = DR2_DONE;
        dr2StartSegment();
      } else {
        L_des = dr2_baseRPM_drive;
        R_des = dr2_baseRPM_drive;
      }
      break;

    case DR2_DONE:
    default:
      L_des = 0; R_des = 0;
      return false;
  }

  return true;
}

// ============================================================================
// VIVE DEAD RECKON (VIVE_DR) -- based on your snippet; signs preserved
//   - Captures (robotX, robotY) at start and computes tick goals once.
//   - Uses your formulas (with variable name fix so it compiles).
// ============================================================================
enum VDRState { VDR_DRIVE1, VDR_TURN1, VDR_DRIVE2, VDR_DONE };
static VDRState vdrState = VDR_DRIVE1;

static long vdrSegStartL = 0, vdrSegStartR = 0;

// Tunables
static float vdr_baseRPM_drive = 450.0f;
static float vdr_baseRPM_turn  = 250.0f;

// Captured start pose
static float xIN = 0.0f;
static float yIN = 0.0f;

// Computed tick goals (set on start)
static long VDR_DRIVE1_TICKS = 0;
static long VDR_TURN1_TICKS  = 0;
static long VDR_DRIVE2_TICKS = 862; // from your snippet

static inline long vdr_avgTicks(long dL, long dR){ return (labs(dL) + labs(dR)) / 2; }
static inline long vdr_turnTicks(long dL, long dR){ return (labs(dL) + labs(dR)) / 2; }

static inline void vdrStartSegment(){
  noInterrupts();
  vdrSegStartL = L_dead;
  vdrSegStartR = R_dead;
  interrupts();
  resetPIDState(pidL);
  resetPIDState(pidR);
}

void viveDRStart(){
  // capture current pose (from vive update running in loop)
  xIN = robotX;
  yIN = robotY;

  // Your formulas (kept as-is, just made identifiers consistent)
  VDR_DRIVE1_TICKS = (long)(fabsf(xIN - 5240.0f) * 43.1f / 118.442f);
  VDR_TURN1_TICKS  = (long)(fabsf(yIN - 1820.0f) * 43.1f / 144.57f);
  VDR_DRIVE2_TICKS = 862;

  vdrState = VDR_DRIVE1;
  vdrStartSegment();

  Serial.print("[VIVE_DR] start xIN="); Serial.print(xIN,1);
  Serial.print(" yIN="); Serial.print(yIN,1);
  Serial.print(" D1ticks="); Serial.print(VDR_DRIVE1_TICKS);
  Serial.print(" T1ticks="); Serial.print(VDR_TURN1_TICKS);
  Serial.print(" D2ticks="); Serial.println(VDR_DRIVE2_TICKS);
}

bool viveDRCompute(float &L_des, float &R_des){
  noInterrupts();
  long curL = L_dead;
  long curR = R_dead;
  interrupts();

  long dL = curL - vdrSegStartL;
  long dR = curR - vdrSegStartR;

  switch(vdrState){

    case VDR_DRIVE1:
      if(vdr_avgTicks(dL, dR) >= VDR_DRIVE1_TICKS){
        L_des = 0; R_des = 0;
        vdrState = VDR_TURN1;
        vdrStartSegment();
      } else {
        L_des = vdr_baseRPM_drive;
        R_des = vdr_baseRPM_drive;
      }
      break;

    case VDR_TURN1:
      if(vdr_turnTicks(dL, dR) >= VDR_TURN1_TICKS){
        L_des = 0; R_des = 0;
        vdrState = VDR_DRIVE2;
        vdrStartSegment();
      } else {
        // KEEP SIGNS AS PROVIDED:
        L_des = vdr_baseRPM_turn;
        R_des = -vdr_baseRPM_turn;
      }
      break;

    case VDR_DRIVE2:
      if(vdr_avgTicks(dL, dR) >= VDR_DRIVE2_TICKS){
        L_des = 0; R_des = 0;
        vdrState = VDR_DONE;
        vdrStartSegment();
      } else {
        L_des = vdr_baseRPM_drive;
        R_des = vdr_baseRPM_drive;
      }
      break;

    case VDR_DONE:
    default:
      L_des = 0; R_des = 0;
      return false;
  }

  return true;
}

// ===================== setup =====================
void setup(){
  Serial.begin(115200);

  pinMode(L_IN1,OUTPUT);
  pinMode(L_IN2,OUTPUT);
  pinMode(R_IN3,OUTPUT);
  pinMode(R_IN4,OUTPUT);
  digitalWrite(L_IN1,LOW);
  digitalWrite(L_IN2,LOW);
  digitalWrite(R_IN3,LOW);
  digitalWrite(R_IN4,LOW);

  ledcAttach(L_EN, PWM_FREQ, PWM_BITS);
  ledcAttach(R_EN, PWM_FREQ, PWM_BITS);
  ledcWrite(L_EN,0);
  ledcWrite(R_EN,0);

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  Serial.print("Connecting");
  unsigned long t0 = millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-t0<10000){
    delay(300);
    Serial.print(".");
  }
  if(WiFi.status()==WL_CONNECTED){
    Serial.printf("\nIP: http://%s\n", WiFi.localIP().toString().c_str());
    delay(2000);
  }else{
    Serial.println("\nWiFi connect failed (offline mode).");
  }

  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), isrR, CHANGE);

  ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_BITS);
  ledcWrite(SERVO_PIN, (uint32_t)(0.05f * SERVO_MAX_DUTY));
  Wire.begin(TOPHAT_SDA, TOPHAT_SCL, 40000);
  Serial.printf("I2C Master init. SDA=%d, SCL=%d, addr=0x%02X\n",
                TOPHAT_SDA, TOPHAT_SCL, TOPHAT_ADDR);

  WireToF.begin(SDA_PIN, SCL_PIN);
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  Serial.println("Starting TOF sensors...");
  setID();
  Serial.println("TOF sensors ready.");

  server.on("/",    handleRoot);
  server.on("/base",handleBase);
  server.on("/cmd", handleCmd);
  server.on("/stat",handleStat);
  server.begin();

  lastCmdMs = millis();

  // ===== Vive initialization =====
  // vive1.begin();
  // vive2.begin();
  // Serial.println("Vive trackers started, waiting for lighthouse lock...");
}

// ===================== main loop =====================
void loop(){
  server.handleClient();

  static unsigned long last = millis();
  unsigned long now = millis();

  if(now - last >= CTRL_DT_MS){
    float dt = (now - last) / 1000.0f;
    last = now;

    noInterrupts();
    long Lt = L_ticks; L_ticks = 0;
    long Rt = R_ticks; R_ticks = 0;
    interrupts();

    float L_rpm = ((Lt/dt)/TICKS_PER_REV_L)*60.0f;
    float R_rpm = ((Rt/dt)/TICKS_PER_REV_R)*60.0f;

    if(L_rpm < 0) L_rpm = 0;
    if(R_rpm < 0) R_rpm = 0;

    lastLrpm = L_rpm;
    lastRrpm = R_rpm;

    int base = g_base;
    float baseL = MAX_RPM_L * (base / 100.0f);
    float baseR = MAX_RPM_R * (base / 100.0f);

    float L_des = 0.0f;
    float R_des = 0.0f;

    // ---- Mode selection: DR1 / DR2 / VIVE_DR / wall-follow / manual ----
    if (g_dr1) {
      bool running = dr1Compute(L_des, R_des);
      if(!running){
        g_dr1 = false;
        g_mode = STOP;
        L_des = 0; R_des = 0;
      }
    }
    else if (g_dr2) {
      bool running = dr2Compute(L_des, R_des);
      if(!running){
        g_dr2 = false;
        g_mode = STOP;
        L_des = 0; R_des = 0;
      }
    }
    else if (g_viveDR) {
      bool running = viveDRCompute(L_des, R_des);
      if(!running){
        g_viveDR = false;
        g_mode = STOP;
        L_des = 0; R_des = 0;
      }
    }
    else if (g_wallFollow) {
      wallFollowCompute(baseL, baseR, L_des, R_des);
    }
    else {
      L_des = baseL;
      R_des = baseR;
      switch(g_mode){
        case STOP:  L_des = 0; R_des = 0; break;
        case FWD:   break;
        case BACK:  L_des = -L_des; R_des = -R_des; break;
        case LEFT:  L_des = 0; break;
        case RIGHT: R_des = 0; break;
      }
    }

    bool  L_fwd     = (L_des >= 0);
    bool  R_fwd     = (R_des >= 0);
    float L_des_mag = fabs(L_des);
    float R_des_mag = fabs(R_des);

    int uL = (L_des_mag < 1.0f) ? 0 : pidcontrol(pidL, L_des_mag, L_rpm, dt);
    int uR = (R_des_mag < 1.0f) ? 0 : pidcontrol(pidR, R_des_mag, R_rpm, dt);

    setLeft (uL, L_fwd);
    setRight(uR, R_fwd);
  }

  if(millis() - last_i2c_time >= I2C_PERIOD_MS){
    last_i2c_time = millis();
    sendTopHatUpdate();
    Serial.printf("[I2C] sent %u pkts, health=%u\n",
                  (unsigned)wifi_packets, (unsigned)current_health);
    wifi_packets = 0;
  }


  // watchdog placeholder (kept from your structure)
  if(millis() - lastCmdMs > WD_MS){
    lastCmdMs = millis();
  }

  // ===== Vive update (UNCOMMENTED) =====
  // static uint32_t lastViveMs = 0;
  // if (millis() - lastViveMs >= 50) {
  //   lastViveMs = millis();

  //   bool ok1 = updateVive(vive1, vive1_x, vive1_y, 5);
  //   bool ok2 = updateVive(vive2, vive2_x, vive2_y, 5);
  // }
}
