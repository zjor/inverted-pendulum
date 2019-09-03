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

volatile long encoderValue = 0;
volatile long lastEncoded = 0;
long lastEncoderValue = 0;

unsigned long lastTimeMillis = 0;

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
  lastTimeMillis = millis();
}

void loop() {
  unsigned long now = millis();

  u = amp * sin(w * now / 1000);
  digitalWrite(DIR_PIN, u > 0.0 ? LOW : HIGH);
  analogWrite(PWM_PIN, fabs(u));  

  float dt = 1.0 * (now - lastTimeMillis) / 1000.0;
  float v = 1.0 * (encoderValue - lastEncoderValue) / dt / PPR;

  Serial.print(u);
  Serial.print("\t");
  Serial.println(v, 4);

  lastTimeMillis = now;
  lastEncoderValue = encoderValue;

  delay(25);
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