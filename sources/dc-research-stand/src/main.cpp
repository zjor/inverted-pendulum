/**
 * OMRON E6B2-CWZ6C pinout
 * - Brown - Vcc
 * - Black - Phase A
 * - White - Phase B
 * - Orange - Phaze Z
 * - Blue - GND
 *
 * LPD3806-600BM-G5-24C pinout
 * - Green - Phase A
 * - White - Phase B
 * - Red - Vcc
 * - Black - GND
 */

#include <Arduino.h>

// encoder pins
#define OUTPUT_A  2
#define OUTPUT_B  3

// pulses per revolution
#define PPR  2400
#define SHAFT_R 0.00611

#define PWM_PIN 5
#define DIR_PIN 4

#define MAX_STALL_U 8.0

#define Kp  80.0
#define Kd  20.0
#define Ki  30.0

volatile long encoderValue = 0;
volatile long lastEncoded = 0;

unsigned long lastTimeMillis = 0;

unsigned long lastTargetChange = 0;

const long initialDelay = 2000;

float amp = 25;
float u = 0.0;
float w = 2.0 * PI / 10.0;

void encoderHandler();

void setup() {
  TCCR1B = TCCR1B & 0b11111000 | 1;  // set 31KHz PWM to prevent motor noise
  pinMode(OUTPUT_A, INPUT_PULLUP);
  pinMode(OUTPUT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(OUTPUT_A), encoderHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(OUTPUT_B), encoderHandler, CHANGE);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(DIR_PIN, LOW);

  Serial.begin(9600);
  lastTimeMillis = 0L;
  lastTargetChange = millis();
}


float x = 0.0;
float last_x = 0.0;
float target_x = 0.5;

float v = 0.0;
float dt = 0.0;
float error = 0.0;
float last_error = 0.0;
float integral_error = 0.0;
float target_v = 0.1;

void setRandomTarget(unsigned long now) {
  if (now - lastTargetChange >= 3000) {
    target_v = 1.0 * random(5) / 10.0;
    target_x = 1.0 * random(5) / 10.0;
    lastTargetChange = now;
  }
}

float avoidStall(float u) {
  if (fabs(u) < MAX_STALL_U) {
    return u > 0 ? MAX_STALL_U : -MAX_STALL_U;
  }
  return u;
}

float getSineControl(unsigned long now) {
  return amp * sin(w * now / 1000);
}

float getVelocityControl(float v, float dt) {
  if (v != v) {
    // v is nan
    return 0.0;
  }

  error = v - target_v;
  float de = (error - last_error) / dt;
  integral_error += error * dt;
  last_error = error;
  return - (Kp * error + Kd * de + Ki * integral_error);
}

float getPositionControl(float x, float dt) {
  if (x != x) {
    return 0.0;
  }

  error = x - target_x;
  float de = (error - last_error) / dt;
  integral_error += error * dt;
  last_error = error;
  return - (Kp * error + Kd * de + Ki * integral_error);
}

void loop() {

  unsigned long now = millis();
  dt = 1.0 * (now - lastTimeMillis) / 1000.0;
  x = 2.0 * PI * encoderValue / PPR * SHAFT_R;
  v = 1.0 * (x - last_x) / dt;

  setRandomTarget(now);

  float orig_u = getPositionControl(x, dt);

  // u = avoidStall(orig_u);
  u = orig_u;

  digitalWrite(DIR_PIN, u > 0.0 ? LOW : HIGH);
  analogWrite(PWM_PIN, fabs(u));  

  // Serial.print(orig_u);
  // Serial.print("\t");

  // Serial.print(u);
  // Serial.print("\t");
  Serial.print(target_x * 100);
  Serial.print("\t");
  // Serial.print(v, 4);
  // Serial.print("\t");
  Serial.println(x * 100);

  lastTimeMillis = now;
  last_x = x;

  delay(10);
}

void encoderHandler() {
  int MSB = (PIND & (1 << OUTPUT_A)) >> OUTPUT_A; //MSB = most significant bit
  int LSB = (PIND & (1 << OUTPUT_B)) >> OUTPUT_B; //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderValue++; //CW
  }
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderValue--; //CCW
  }

  lastEncoded = encoded; //store this value for next time  
}