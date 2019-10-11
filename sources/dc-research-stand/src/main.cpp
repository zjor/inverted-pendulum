/**
 * == DC motor research stand ==
 * 
 * This software is used to collect data on DC motor acceleration profile in order to estimate
 * motor's constants for further usage in control applications.
 * 
 * == Hardware specification ==
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

#define STATE_PENDING_COMMAND 0
#define STATE_RUNNING         1
#define STATE_HOMING          2

#define DATA_BUF_SIZE 500

volatile long encoderValue = 0L;
volatile long lastEncoded = 0L;
long startEncoderValue = 0L;

unsigned long lastTimeMillis = 0L;
unsigned long startTimeMicros = 0L;

unsigned long times[DATA_BUF_SIZE];
unsigned long values[DATA_BUF_SIZE];
int index = 0;


float amp = 255;
float u = 0.0;
float w = 0.0;
float dt = 0.0;

float x, last_x;

unsigned int state = STATE_PENDING_COMMAND;

char buf[8];
int buf_pos = 0;
int pwm = 0;

PID cartPID(Kp, Kd, Ki, 0.);

void encoderHandler();

void setup() {

  // setting PWD frequency on pin 10 to 31kHz
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;

  pinMode(OUTPUT_A, INPUT_PULLUP);
  pinMode(OUTPUT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(OUTPUT_A), encoderHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(OUTPUT_B), encoderHandler, CHANGE);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(DIR_PIN, LOW);

  Serial.begin(9600);
  Serial.println("Enter PWM");
  lastTimeMillis = 0L;
}

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

float getCartDistance(long pulses, long ppr) {
  return 2.0 * PI * pulses / PPR * SHAFT_R;
}

void clear_buf(char *buf, int len) {
    int i = 0;
    while (i < len) {
        buf[i++] = 0;
    }
}

void driveMotor(float u) {
  digitalWrite(DIR_PIN, u > 0.0 ? LOW : HIGH);
  analogWrite(PWM_PIN, fabs(u));
}

void startMeasurements(int pwm) {
  Serial.print("\nRunning at PWM: "); Serial.println(pwm);
  index = 0;
  startTimeMicros = micros();
  startEncoderValue = encoderValue;
  
  state = STATE_RUNNING;

  driveMotor(pwm);    
}

void loop() {    
    x = getCartDistance(encoderValue, PPR);

    if (state == STATE_PENDING_COMMAND) {
        if (Serial.available() > 0) {
            int c = Serial.read();
            if (isDigit(c) && buf_pos < 8) {
                buf[buf_pos++] = (char)c;
                Serial.print((char)c);
            } else if (c == '\n') {
                pwm = atoi(buf);
                buf_pos = 0;
                clear_buf(buf, 8);
                startMeasurements(pwm);
            }
        }
    } else if (state == STATE_RUNNING) {
        unsigned long nowMicros = micros();
        unsigned long ticks = nowMicros - startTimeMicros;
        long measurement = encoderValue - startEncoderValue;

        times[index] = ticks;
        values[index] = measurement;
        index++;

        if (fabs(x) > POSITION_LIMIT || ticks > 5000000 || index > DATA_BUF_SIZE) {
            driveMotor(0);
            
            for (int i = 0; i < index; i++) {
              Serial.print(i);Serial.print(",");
              Serial.print(pwm);Serial.print(",");
              Serial.print(times[i]);Serial.print(",");
              Serial.println(getCartDistance(values[i], PPR), 6);
            }

            state = STATE_HOMING;
            Serial.println("Homing...");
        }
        delay(5);
    } else if (state == STATE_HOMING) {
        if (fabs(x) <= 0.005) {
          driveMotor(0);
          state = STATE_PENDING_COMMAND;
          Serial.println("Homing complete\nEnter PWM");
          return;
        }
        unsigned long now = millis();
        dt = 1.0 * (now - lastTimeMillis) / 1000.0;
        lastTimeMillis = now;
        u = cartPID.getControl(x, last_x, dt);
        driveMotor(saturate(avoidStall(u), 240));
        delay(5);
    }
    
    last_x = x;    
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