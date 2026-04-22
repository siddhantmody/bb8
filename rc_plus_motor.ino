#include <Arduino.h>
#include <ESP32Servo.h>

// --- PIN DEFINITIONS ---
// Encoder Pins
const int ENC1 = 36; //driving  
const int ENC2 = 37; // spinning  

// RC Input Pins
const int RC_CH1_IN = 4; 
const int RC_CH2_IN = 5;
const int RC_CH3_IN = 6;
const int RC_CH4_IN = 7;

// Motor Pins (M5Stack Stamp C3)
const int DIR1 = 40; const int PWM1 = 39;
const int DIR2 = 41; const int PWM2 = 38;
const int SERVO = 3;

// --- STATE VARIABLES ---
volatile uint32_t enc1_ticks = 0, enc2_ticks = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// --- INTERRUPT SERVICE ROUTINES ---
// These just count raw pulses now
void IRAM_ATTR isr1() { enc1_ticks++; }
void IRAM_ATTR isr2() { enc2_ticks++; }

void setup() {
  Serial.begin(115200);
  
  // Motor Pins
  pinMode(DIR1, OUTPUT); pinMode(DIR2, OUTPUT);
  ledcAttach(PWM1, 5000, 8); // 8-bit resolution (0-255)
  ledcAttach(PWM2, 5000, 8);

  // RC Inputs
  pinMode(RC_CH1_IN, INPUT);
  pinMode(RC_CH2_IN, INPUT);

  // Encoder Inputs
  pinMode(ENC1, INPUT_PULLUP);
  pinMode(ENC2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1), isr1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2), isr2, RISING);

  Serial.println("RC Control & Raw Encoder Logging Ready.");
}

void driveMotor(int pinPWM, int pinDIR, int speed) {
  // Sets direction and magnitude
  digitalWrite(pinDIR, speed >= 0 ? HIGH : LOW);
  ledcWrite(pinPWM, constrain(abs(speed), 0, 255));
}

void loop() {
  // 1. RC CONTROL LOGIC
  // Capture pulses (timeout 25ms)
  long pulse1 = pulseIn(RC_CH1_IN, HIGH, 25000);
  long pulse2 = pulseIn(RC_CH2_IN, HIGH, 25000);

  // Map to motor range (-255 to 255)
  // Check if signal exists (pulse > 0) to prevent ghost movement
  int motor1_speed = (pulse1 > 0) ? map(pulse1, 1100, 1900, -255, 255) : 0;
  int motor2_speed = (pulse2 > 0) ? map(pulse2, 1100, 1900, 255, -255) : 0;

  // Apply Deadzone (10% of range)
  if (abs(motor1_speed) < 25) motor1_speed = 0;
  if (abs(motor2_speed) < 25) motor2_speed = 0;

  // Update Motors
  driveMotor(PWM1, DIR1, motor1_speed);
  driveMotor(PWM2, DIR2, motor2_speed);

  // 2. LOGGING DATA
  // We'll read and reset the counters to see "ticks per loop"
  uint32_t snapshot1, snapshot2;
  portENTER_CRITICAL(&mux);
  snapshot1 = enc1_ticks; enc1_ticks = 0;
  snapshot2 = enc2_ticks; enc2_ticks = 0;
  portEXIT_CRITICAL(&mux);

  // Output: Pulse Durations and Tick Counts
  if (millis() % 50 == 0) { // Log at 20Hz
    Serial.printf("RC:[%ld, %ld] | EncTicks:[%u, %u]\n", pulse1, pulse2, snapshot1, snapshot2);
  }

  delay(10); 
}