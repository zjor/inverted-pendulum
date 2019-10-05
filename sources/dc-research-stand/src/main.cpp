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

// motor encoder pins
#define OUTPUT_A  3 // PE5
#define OUTPUT_B  2 // PE4

// reference encoder pins
#define REF_OUT_A 18 // PD3
#define REF_OUT_B 19 // PD2

// pulses per revolution
#define PPR  2400
#define SHAFT_R 0.00573

#define PWM_PIN 10
#define DIR_PIN 8

#define MAX_STALL_U 30.0
#define POSITION_LIMIT  0.2

#define Kp  10000.0
#define Kd  600.0
#define Ki  2000.0

volatile long encoderValue = 0;
volatile long lastEncoded = 0;

volatile long refEncoderValue = 0;
volatile long lastRefEncoded = 0;

unsigned long lastTimeMillis = 0;

float amp = 255;
float u = 0.0;
float w = 0.0;

void encoderHandler();
void refEncoderHandler();

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
float error = 0.0;
float last_error = 0.0;
float integral_error = 0.0;

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

float getSineControl(unsigned long now) {
  return amp * sin(w * now / 1000);
}

float getPIDControl(float value, float lastValue, float target, float dt) {
  error = target - value;
  float de = -(value - lastValue) / dt;
  integral_error += error * dt;
  last_error = error;
  return (Kp * error + Kd * de + Ki * integral_error);
}

float getAngle(long pulses, long ppr) {
  return 2.0 * PI * pulses / ppr;
}

float getCartDistance(long pulses, long ppr) {
  return 2.0 * PI * pulses / PPR * SHAFT_R;
}

float last_x = 0.;
float set_point = 0.;

unsigned long i = 0;

float step_u = 254.0;
boolean is_measuring = true;

void loop() {
  unsigned long now = millis();
  dt = 1.0 * (now - lastTimeMillis) / 1000.0;

  float x = getCartDistance(encoderValue, PPR);
  float v = (x - last_x) / dt;
  if (is_measuring && fabs(x) < POSITION_LIMIT) {
    u = step_u;
    if (fabs(x) > 0.17) {
      is_measuring = false;
    }
  } else {
    float orig_u = getPIDControl(x, last_x, set_point, dt);
    u = saturate(avoidStall(orig_u), 255.0);
  }
  
  last_x = x;

  if (fabs(x) > POSITION_LIMIT) {
    u = 0;
  }

  digitalWrite(DIR_PIN, u > 0.0 ? LOW : HIGH);
  analogWrite(PWM_PIN, fabs(u));  

  if (is_measuring) {
    Serial.print(now);
    Serial.print("\t");
    Serial.print(u);
    Serial.print("\t");
    Serial.print(v, 4);
    Serial.print("\t");
    Serial.println(x * 100);
  }
  i++;

  lastTimeMillis = now;

  set_point = 2.0 * PI * refEncoderValue / 5000 * SHAFT_R;

  delay(5);
}

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