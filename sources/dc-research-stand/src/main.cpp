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

#define PWM_PIN 5
#define DIR_PIN 4

#define MAX_STALL_U 7.0

#define Kp  3.0
#define Kd  0.0
#define Ki  1.0

volatile long encoderValue = 0;
volatile long lastEncoded = 0;
long lastEncoderValue = 0;

unsigned long lastTimeMillis = 0;

unsigned long lastVelocityChange = 0;

const long initialDelay = 2000;

float amp = 25;
float u = 0.0;
float w = 2.0 * PI / 10.0;

void encoderHandler();

void setup() {
  pinMode(OUTPUT_A, INPUT_PULLUP);
  pinMode(OUTPUT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(OUTPUT_A), encoderHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(OUTPUT_B), encoderHandler, CHANGE);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(DIR_PIN, LOW);

  Serial.begin(9600);
  lastTimeMillis = 0L;
  lastVelocityChange = millis();
}

float v = 0.0;
float dt = 0.0;
float error = 0.0;
float last_error = 0.0;
float integral_error = 0.0;
float target_v = 4.5;

void setRandomTargetVelocity(unsigned long now) {
  if (now - lastVelocityChange >= 3000) {
    target_v = random(3, 6);
    lastVelocityChange = now;
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

void loop() {

  unsigned long now = millis();
  dt = 1.0 * (now - lastTimeMillis) / 1000.0;
  v = 1.0 * (encoderValue - lastEncoderValue) / dt / PPR;

  setRandomTargetVelocity(now);

  float orig_u = getVelocityControl(v, dt);

  u = avoidStall(orig_u);

  digitalWrite(DIR_PIN, u > 0.0 ? LOW : HIGH);
  analogWrite(PWM_PIN, fabs(u));  

  Serial.print(orig_u);
  Serial.print("\t");

  Serial.print(u);
  Serial.print("\t");
  Serial.print(target_v);
  Serial.print("\t");
  Serial.println(v, 4);

  lastTimeMillis = now;
  lastEncoderValue = encoderValue;

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