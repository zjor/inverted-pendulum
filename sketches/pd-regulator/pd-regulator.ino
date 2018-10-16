/**
 * This sketch demonstrates how to bring a cart to [x: 0; v: 0] from non-zero position using PD-regulator
 * 
 * Dependencies:
 * - CircularBuffer
 */
#include <float.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <CircularBuffer.h>
 
#define STEP_PIN    9
#define DIR_PIN     10
#define PULSE_WIDTH 5

#define POSITION_LIMIT  1500

#define ADS0_MIN  242
#define ADS0_MAX  24141

#define ANGLE_ZERO  3.650
#define V0  0.0
#define P0  0
// -26.22131018  -7.50023911  -0.4472136   -1.30242157

#define Kp  0.447
#define Kd  1.302

#define aKp  260.221
#define aKd  70.5

#define ANGLE_SMOOTHING_WINDOW  9

//const float g7[] = { 0.03559661344, 0.1103628062, 0.2176061504, 0.27286886, 0.2176061504, 0.1103628062, 0.03559661344 };
const float g9[] = {0.02376257744, 0.06195534498, 0.1228439993, 0.185233293, 0.2124095706, 0.185233293, 0.1228439993, 0.06195534498, 0.02376257744 };

// angle values
CircularBuffer<float, ANGLE_SMOOTHING_WINDOW> thetas;

// angular velocity values
CircularBuffer<float, ANGLE_SMOOTHING_WINDOW> omegas;

Adafruit_ADS1115 angleSensor(0x48);

float a = 0.0; // steps per second^2
float v = V0; //steps per second
float f = 1000000.0; //micros per second

float angle = FLT_MIN;
float lastAngle = FLT_MIN;
float omega = FLT_MIN;
unsigned long lastAngleUpdate = 0;
unsigned long angleUpdateInterval = f / 100;

unsigned long lastSpeedUpdate = 0;
unsigned long speedUpdateInterval = f / 100;

int direction = HIGH;
int position = P0;

unsigned long stepDelay = 0;
unsigned long lastStepTime = 0;

void setup() {  
  pinMode(STEP_PIN, OUTPUT); 
  pinMode(DIR_PIN, OUTPUT);
  angleSensor.begin();
  
  setSpeed(v);

//  Serial.begin(115200);
}

unsigned long i = 0;

void loop() {  
  unsigned long now = micros();
  if (now - lastAngleUpdate >= angleUpdateInterval) {
    angle = readAngle();
    if (angle != FLT_MIN && lastAngle != FLT_MIN) {
      float tomega = (angle - lastAngle) * f / (now - lastAngleUpdate);
      omegas.push(tomega);
      if (omegas.isFull()) {
        omega = smooth9(omegas);
        calculateAcceleration(angle, omega, position, v);
        updateSpeedAndStepDelay(now);
      }
    }
    lastAngle = angle;
    lastAngleUpdate = now;    
  }
  
  now = micros();
  if (now - lastStepTime >= stepDelay) {    
    if (stepDelay > PULSE_WIDTH) {
      step();
    }
    
    lastStepTime = now;

//    if (i%10 == 0) {
//      Serial.print(angle, 4);
//      Serial.print("\t\t");
//      Serial.print(omega, 4);
//      Serial.print("\t\t");
//      Serial.print((float(position) / 10000.0), 4);
//      Serial.print("\t\t");
//      Serial.print(10000.0 * v, 4);
//      Serial.print("\t\t");
//      Serial.println(a, 4);  
//    }
//    i++;     
  }

}

float readAngle() {
  int16_t adc0 = angleSensor.readADC_SingleEnded(0);
  float angle = mapf(adc0, ADS0_MIN, ADS0_MAX, 0.0, 2.0 * PI) - ANGLE_ZERO;
  thetas.push(angle);
  if (thetas.isFull()) {
    return smooth9(thetas);
  } else {
    return FLT_MIN;
  }
}

void calculateAcceleration(float theta, float omega, int position, float v) {
  float x = float(position) / 10000.0;
  a = - (Kp * x + Kd * v + theta * aKp + omega * aKd);
}

void updateSpeedAndStepDelay(unsigned long now) {
  if (now - lastSpeedUpdate >= speedUpdateInterval) {
    v += a * (now - lastSpeedUpdate) / f;
    lastSpeedUpdate = now;
    setSpeed(1000.0 * v);
  }  
}

void setSpeed(float speed) {  
  direction = (speed > 0.0) ? HIGH : LOW;  
  stepDelay = (speed == 0) ? 0 : (f / fabs(speed) - PULSE_WIDTH);
}

void step() {
  if (abs(position) > POSITION_LIMIT) {
    return; 
  }
  
  digitalWrite(DIR_PIN, direction);
  digitalWrite(STEP_PIN, HIGH); 
  delayMicroseconds(PULSE_WIDTH); 
  digitalWrite(STEP_PIN, LOW);
  position += (direction == HIGH) ? 1 : -1;
}

float smooth9(CircularBuffer<float, ANGLE_SMOOTHING_WINDOW> &buf) {  
  float avg = .0;
  for (long i = 0; i < ANGLE_SMOOTHING_WINDOW; i++) {
    avg += g9[i] * buf[i];
  }  
  return avg;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
