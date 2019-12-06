/**
 * == Inverted pendulum stabilisation with state control with DC motor ==
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

// motor encoder pins
#define OUTPUT_A  3 // PE5
#define OUTPUT_B  2 // PE4

// pendulum encoder pins
#define REF_OUT_A 18 // PD3
#define REF_OUT_B 19 // PD2

// terminal switches pins
#define SWITCH_RIGHT  21
#define SWITCH_LEFT  20

// pulses per revolution
#define PPR  2400
#define SHAFT_R 0.00573
#define PENDULUM_ENCODER_PPR  10000

#define PWM_PIN 10
#define DIR_PIN 8

#define POSITION_LIMIT  0.5 //0.145

#define STATE_CALIBRATE 0
#define STATE_SWING_UP  1
#define STATE_BALANCE   2
#define STATE_FULL_STOP 3
#define STATE_GO_RIGHT  4 
#define STATE_GO_LEFT   5

#define A 35.98
#define B 2.22
#define C 2.79

#define Kth 147.1
#define Kw  36.0
#define Kx  54.0
#define Kv  39.5

const float THETA_THRESHOLD = PI / 12;
const float PI2 = 2.0 * PI;

volatile long encoderValue = 0L;
volatile long lastEncoded = 0L;

volatile long refEncoderValue = 0;
volatile long lastRefEncoded = 0;

volatile boolean leftSwitchPressed = false;
volatile boolean rightSwitchPressed = false;

unsigned long now = 0L;
unsigned long lastTimeMicros = 0L;

// pulses count before the right switch pressed
long rightSwitchPulses;

float x, last_x, v, dt;
float theta, last_theta, w;
float control, u;

unsigned long log_prescaler = 0;

// int state = STATE_CALIBRATE;
int state = STATE_GO_RIGHT;

void encoderHandler();
void refEncoderHandler();

void leftSwitchHandler();
void rightSwitchHandler();

void calibrate();

void setup() {

  // setting PWD frequency on pin 10 to 31kHz
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;

  pinMode(OUTPUT_A, INPUT_PULLUP);
  pinMode(OUTPUT_B, INPUT_PULLUP);

  pinMode(REF_OUT_A, INPUT_PULLUP);
  pinMode(REF_OUT_B, INPUT_PULLUP);

  pinMode(SWITCH_LEFT, INPUT);
  pinMode(SWITCH_RIGHT, INPUT);

  attachInterrupt(digitalPinToInterrupt(OUTPUT_A), encoderHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(OUTPUT_B), encoderHandler, CHANGE);

  attachInterrupt(digitalPinToInterrupt(REF_OUT_A), refEncoderHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REF_OUT_B), refEncoderHandler, CHANGE);

  attachInterrupt(digitalPinToInterrupt(SWITCH_LEFT), leftSwitchHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(SWITCH_RIGHT), rightSwitchHandler, FALLING);


  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(DIR_PIN, LOW);

  Serial.begin(9600);
  lastTimeMicros = 0L;
}

float saturate(float v, float maxValue) {
  if (fabs(v) > maxValue) {
    return (v > 0) ? maxValue : -maxValue;
  } else {
    return v;
  }
}

float getAngle(long pulses, long ppr) {  
  float angle = (PI + PI2 * pulses / ppr);
  while (angle > PI) {
    angle -= PI2;
  }
  while (angle < -PI) {
    angle += PI2;
  }
  return angle;
}

float getCartDistance(long pulses, long ppr) {
  return 2.0 * PI * pulses / PPR * SHAFT_R;
}

void driveMotor(float u) {
  digitalWrite(DIR_PIN, u > 0.0 ? LOW : HIGH);
  analogWrite(PWM_PIN, fabs(u));
}

boolean isControllable(float theta) {
  return fabs(theta) < THETA_THRESHOLD;
}

void log_state(float control, float u) {
  if (fabs(w) > 100) {
    return;
  }

  if (state == STATE_FULL_STOP) {
    return;
  }

  // if (log_prescaler % 5 == 0) {
    Serial.print(state);Serial.print("\t");
    Serial.print(theta, 4);Serial.print("\t");
    Serial.print(w, 4);Serial.print("\t");
    Serial.print(x, 4);Serial.print("\t");
    Serial.print(v, 4);Serial.print("\t");
    Serial.print(control, 4);Serial.print("\t");
    Serial.println(u, 4);
  // }
  log_prescaler++;
}

float getBalancingControl(float x, float v, float theta, float w) {
  return Kx * x + Kv * v + Kth * theta + Kw * w;
}

float getSwingUpControl(float x, float v, float theta, float w) {
  float K = 12.0;
  float c = -copysignf(K, - w * cos(theta));
  float lim = 0.01;
  if ((x >= lim && c > 0) || (x <= -lim && c < 0) || fabs(theta) < PI / 2) {
    c = -(A * v + copysignf(C, v));
  }
  return c;
}

float integralError = 0.0;
float lastTarget = 0.0;
float getVelocityControl(float v, float target, float dt) {

  if (lastTarget != target) {
    lastTarget = target;
    integralError = 0.0;
  }

  float error = target - v;
  float Kp = 50.0;
  float Ki = 10.0;
  integralError += Ki * error * dt;
  return Kp * error + integralError;
}

void driveMotorWithControl(float control, float v) {
  u = (control + A * v + copysignf(C, v)) / B;
  u = 255.0 * u / 12.0;
  driveMotor(saturate(u, 254));
}

void loop() {

  if (leftSwitchPressed) {
    leftSwitchPressed = false;
    Serial.println("The cart has reached the left end");
    long lengthPulses = rightSwitchPulses - encoderValue;
    float railLength = getCartDistance(lengthPulses, PPR);
    Serial.print(lengthPulses); Serial.print("\t");
    Serial.println(railLength);
    state = STATE_FULL_STOP;   
  }

  if (rightSwitchPressed) {
    rightSwitchPressed = false;
    rightSwitchPulses = encoderValue;
    Serial.println("The cart has reached the right end");
    if (state == STATE_GO_RIGHT) {
      state = STATE_GO_LEFT;
      Serial.println("Going left...");
    } else {
      state = STATE_FULL_STOP;
      Serial.println("Unexpected state. Halt.");
    }    
  }

  now = micros();
  dt = 1.0 * (now - lastTimeMicros) / 1000000;
  x = getCartDistance(encoderValue, PPR);
  v = (x - last_x) / dt;

  theta = getAngle(refEncoderValue, PENDULUM_ENCODER_PPR);
  w = (theta - last_theta) / dt;

  if (fabs(x) >= POSITION_LIMIT) {
    if (state != STATE_FULL_STOP) {
      Serial.println("Cart reached end of rail");
    }
    state = STATE_FULL_STOP;
  }

  switch (state) {
    case STATE_GO_RIGHT:
      control = getVelocityControl(v, 0.06, dt);
      driveMotorWithControl(control, v);
      break;
    case STATE_GO_LEFT:
      control = getVelocityControl(v, -0.06, dt);
      driveMotorWithControl(control, v);
      break;
    case STATE_CALIBRATE:
      Serial.println("Waiting for the pendulum to come to rest...");
      calibrate();
      Serial.println("Pendulum is at rest");
      state = STATE_SWING_UP;
      break;
    case STATE_SWING_UP:      
      control = getSwingUpControl(x, v, theta, w);
      driveMotorWithControl(control, v);
      if (isControllable(theta)) {
        state = STATE_BALANCE;
      }
      break;
    case STATE_BALANCE:
      control = getBalancingControl(x, v, theta, w);
      driveMotorWithControl(control, v);
      break;
    case STATE_FULL_STOP:
      driveMotor(0);
      break;
  }
  
  last_x = x;
  last_theta = theta;
  lastTimeMicros = now;
      
  log_state(control, u);
  
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

/**
 * Waits till pendulum stays at rest in the vertical position
 */
void calibrate() {
  long lastReading;
  do {
    lastReading = refEncoderValue;
    delay(1000);
  } while (lastReading != refEncoderValue);
  cli();
  refEncoderValue = 0;
  sei();
}

void leftSwitchHandler() {
  leftSwitchPressed = true;
}

void rightSwitchHandler() {
  rightSwitchPressed = true;
}