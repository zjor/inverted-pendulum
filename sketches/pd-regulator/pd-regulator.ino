/**
 * This sketch demonstrates how to bring a cart to [x: 0; v: 0] from non-zero position using PD-regulator
 * 
 * Dependencies:
 * - CircularBuffer
 */
#include <CircularBuffer.h>
 
#define STEP_PIN    9
#define DIR_PIN     10
#define PULSE_WIDTH 5

#define POSITION_LIMIT  1500

#define ANGLE_ZERO  522.4
#define V0  0.0
#define P0  0
#define Kp  10.0
#define Kd  8.0

#define aKp  1000000.0
#define aKd  80000.0

double a = 0.0; // steps per second^2
double v = V0; //steps per second
double f = 1000000.0; //micros per second

double angle = 0.0;
double lastAngle = 0.0;
double w = 0.0;

CircularBuffer<double, 4> wAvg;

unsigned long lastAngleUpdate = 0;
unsigned long angleUpdateInterval = f / 20;

unsigned long lastSpeedUpdate = 0;
unsigned long speedUpdateInterval = f / 500;

int direction = HIGH;
int position = P0;

unsigned long stepDelay = 0;
unsigned long lastStepTime = 0;
unsigned long now = 0;

void setup() {  
  pinMode(STEP_PIN, OUTPUT); 
  pinMode(DIR_PIN, OUTPUT);

  setSpeed(v);
  lastAngle = angle = normalizeAngle(analogRead(A0), ANGLE_ZERO);

//  Serial.begin(115200);
}

unsigned long i = 0;

void loop() {  
  now = micros();
  angle = normalizeAngle(analogRead(A0), ANGLE_ZERO);
  
  if (now - lastAngleUpdate >= angleUpdateInterval) {    
    w = (angle - lastAngle) * f / (now - lastAngleUpdate);
    wAvg.push(w);
    lastAngle = angle;
    lastAngleUpdate = now;
    calculateAcceleration();
    updateSpeedAndStepDelay(now);
  }
  
  now = micros();
  if (now - lastStepTime >= stepDelay) {    
    if (stepDelay > PULSE_WIDTH) {
      step();
    }
    
    lastStepTime = now;
    calculateAcceleration();
    updateSpeedAndStepDelay(now);

//    if (i%10 == 0) {
//      Serial.print(angle);
//      Serial.print("\t\t");
//      Serial.println(w, 4);
//      Serial.print("\t\t");
//      Serial.print(position);
//      Serial.print("\t\t");
//      Serial.print(v);
//      Serial.print("\t\t");
//      Serial.println(a);  
//    }
//    i++;     
  }

}

void calculateAcceleration() {
  if (wAvg.isFull()) {
    double w = (wAvg[0] + wAvg[1] + wAvg[2] + wAvg[3]) / 4.0;
    a = - (Kp * position + Kd * v + angle * aKp + w * aKd);
  }
}

void updateSpeedAndStepDelay(unsigned long now) {
  if (now - lastSpeedUpdate >= speedUpdateInterval) {
    v += a * (now - lastSpeedUpdate) / f;
    lastSpeedUpdate = now;
    setSpeed(v);
  }  
}

void setSpeed(int speed) {
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

double dmap(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double normalizeAngle(double angle, double zero) {
  return dmap(angle, 0, 1024, 0.0, 2.0 * PI) - dmap(zero, 0, 1024, 0.0, 2.0 * PI);
}
