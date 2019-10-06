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

#include "PID.h"

// motor encoder pins
#define OUTPUT_A  3 // PE5
#define OUTPUT_B  2 // PE4

// pendulum encoder pins
#define REF_OUT_A 18 // PD3
#define REF_OUT_B 19 // PD2

#define MOTOR_ENCODER_PPR  2400
#define SHAFT_R 0.00573
#define PENDULUM_ENCODER_PPR  10000

#define PWM_PIN 10
#define DIR_PIN 8

#define MAX_STALL_U 40.0
#define POSITION_LIMIT  0.12
#define THETA_THRESHOLD PI/10

#define Kp  1500.0
#define Kd  100.0
#define Ki  200.0

#define thKp  20000.0
#define thKd  500.0
#define thKi  0.0

volatile long encoderValue = 0;
volatile long lastEncoded = 0;

volatile long refEncoderValue = 0;
volatile long lastRefEncoded = 0;

unsigned long lastTimeMillis = 0;

float theta = 0.;
float last_theta = 0.;
float last_w_filtered = 0.;
float filter_alpha = 0.5;

void encoderHandler();
void refEncoderHandler();

PID cartPID(Kp, Kd, Ki, 0.);
PID pendulumPID(thKp, thKd, thKi, 0.);

void setup() {

  // setting PWD frequency on pin 10 to 31kHz
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;

  pinMode(OUTPUT_A, INPUT_PULLUP);
  pinMode(OUTPUT_B, INPUT_PULLUP);

  pinMode(REF_OUT_A, INPUT_PULLUP);
  pinMode(REF_OUT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(OUTPUT_A), encoderHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(OUTPUT_B), encoderHandler, CHANGE);

  attachInterrupt(digitalPinToInterrupt(REF_OUT_A), refEncoderHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REF_OUT_B), refEncoderHandler, CHANGE);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(DIR_PIN, LOW);

  Serial.begin(9600);
  lastTimeMillis = 0L;
}

float dt = 0.0;

float avoidStall(float u) {
  if (fabs(u) < MAX_STALL_U) {
    return u > 0 ? u + MAX_STALL_U : u - MAX_STALL_U;
  }
  return u;
}

float saturate(float v, float maxValue) {
  if (fabs(v) > maxValue) {
    return (v > 0) ? maxValue : -maxValue;
  } else {
    return v;
  }
}

float getAngle(long pulses, long ppr) {
  return PI + 2.0 * PI * pulses / ppr;
}

float getCartDistance(long pulses, long ppr) {
  return 2.0 * PI * pulses / ppr * SHAFT_R;
}

float getControl(float th, float w, float x, float v) {
  return thKp * th + thKd * w;
}

void driveMotor(float u) {
  digitalWrite(DIR_PIN, u > 0.0 ? LOW : HIGH);
  analogWrite(PWM_PIN, fabs(u));
}

boolean isControllable(float theta) {
  return fabs(theta) < THETA_THRESHOLD;
}

float x = 0.;
float last_x = 0.;
float u = 0.;

float is_docking = false;

unsigned long i = 0;

void loop() {
  unsigned long now = millis();
  if (now < 10) {
    return;
  }
  dt = 1.0 * (now - lastTimeMillis) / 1000.0;

  theta = getAngle(refEncoderValue, PENDULUM_ENCODER_PPR);
  float w = (theta - last_theta) / dt;
  float w_filtered = filter_alpha * w + (1.0 - filter_alpha) * last_w_filtered;
  last_theta = theta;
  last_w_filtered = w_filtered;

  x = getCartDistance(encoderValue, MOTOR_ENCODER_PPR);
  float v = (x - last_x) / dt;
  last_x = x;
  if (fabs(x) > POSITION_LIMIT) {
    is_docking = true;
  }

  if (isControllable(theta) && fabs(x) < POSITION_LIMIT && !is_docking) {
    // u = getControl(theta, w_filtered, x, v);
    u = -pendulumPID.getControl(theta, last_theta, dt);
    u -= cartPID.getControl(x, last_x, dt);
    u = saturate(avoidStall(u), 240);
  } else if (is_docking) {
    u = cartPID.getControl(x, last_x, dt);
    u = saturate(avoidStall(u), 240);
  } else {
    u = 0.;
  }
  driveMotor(u);

  if (i % 10 == 0) {
    Serial.print(theta, 4);
    Serial.print("\t");
    Serial.print(w, 4);
    Serial.print("\t");
    Serial.print(x);
    Serial.print("\t");
    Serial.println(u);  
  }
  i++;

  lastTimeMillis = now;

  delay(5);
}

/**
 * Motor encoder handler
 */
void encoderHandler() {
  int MSB = (PINE & (1 << PE5)) >> PE5; //MSB = most significant bit
  int LSB = (PINE & (1 << PE4)) >> PE4; //LSB = least significant bit
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

/**
 * Pendulum encoder handler
 * Encoder attached to pins:
 * Phase A - 18 PD3
 * Phase B - 19 PD2
 */
void refEncoderHandler() {
  int MSB = (PIND & (1 << PD3)) >> PD3; //MSB = most significant bit
  int LSB = (PIND & (1 << PD2)) >> PD2; //LSB = least significant bit
  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastRefEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    refEncoderValue++; //CW
  }
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    refEncoderValue--; //CCW
  }

  lastRefEncoded = encoded; //store this value for next time  
}