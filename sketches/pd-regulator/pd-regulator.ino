/**
 * This sketch demonstrates how to bring a cart to [x: 0; v: 0] from non-zero position using PD-regulator
 */
#include <float.h>
#include <limits.h>
#include <AnalogScanner.h>
#include <CircularBuffer.h>
 
#define STEP_PIN    9
#define DIR_PIN     10
#define PULSE_WIDTH 4

#define POSITION_LIMIT  1500


#define ANGLE_ZERO  497

//-37.15237371  -9.92437418  -0.31622777  -3.42969041
//-34.37967732  -7.76167968  -0.31622777  -3.4035304
// 35 7.5 5.0 2.0

#define aKp  45.0
#define aKd  15.0

#define Kp  0.3
#define Kd  3.4

#define P0  0
#define V0  0.0


const float g9[] = { 0.02376257744, 0.06195534498, 0.1228439993, 0.185233293, 0.2124095706, 0.185233293, 0.1228439993, 0.06195534498, 0.02376257744 };
const float cutoff = 0.5;

AnalogScanner scanner;

volatile boolean adcReady = false;
volatile int adcValue;

CircularBuffer<float, 9> angles;

float a = .0; // meters per second^2
float v = V0; //meters per second
float f = 1000000.0; //micros per second

int direction = HIGH;
long position = P0;

unsigned long stepDelay = 0;
unsigned long lastStepTime = 0;

unsigned long lastEvolutionTime = 0;
unsigned long evolutionPeriod = 5000;

unsigned long lastAngleUpdateTime = 0;
unsigned long angleUpdatePeriod = 40000;
float lastAngle = FLT_MIN;
float filteredAngle = FLT_MIN;
float lastFilteredAngle = FLT_MIN;
float omega = FLT_MIN;

void setup() {  
  pinMode(STEP_PIN, OUTPUT); 
  pinMode(DIR_PIN, OUTPUT);

  scanner.setCallback(A0, onADC);
  int order[] = {A0};  
  scanner.setScanOrder(1, order);
  scanner.beginScanning();
  
//  Serial.begin(115200);
}

long i = 0;

void loop() {
  updateAngleAndDerivative();
  evolveWorld();
  runMotor();

//  if (i % 100 == 0) {
//    Serial.print(filteredAngle, 6);
//    Serial.print("\t");
//    Serial.print(omega, 6);
//    Serial.print("\t");
//    Serial.print(float(position) / 10000.0, 6);
//    Serial.print("\t");
//    Serial.print(v, 6);
//    Serial.print("\t");
//    Serial.println(a, 6);    
//  }
//  i++;
  
}

float getControl(float th, float omega, float x, float v) {
  if (omega == FLT_MIN) {
    return 0.0;
  }
  return - (Kp * x + Kd * v + aKp * th + aKd * omega);
}

void evolveWorld() {
  unsigned long now = micros();
  if (now - lastEvolutionTime >= evolutionPeriod) {
    a = getControl(filteredAngle, omega, float(position) / 10000.0, v);
    
    float dt = float(now - lastEvolutionTime) / f;
    v += a * dt;
    stepDelay = getStepDelay(v);
    lastEvolutionTime = now;
  }
}

//state variables: filteredAngle, omega
void updateAngleAndDerivative() {

  if (adcReady) {      
    angles.push(adcValue);    
    adcReady = false;
  }
  
  unsigned long now = micros();
  if (now - lastAngleUpdateTime >= angleUpdatePeriod) {    
    if (angles.isFull()) {
      float angle = normalizeAngle(smooth9(angles));
      filteredAngle = lastAngle + cutoff * (angle - lastAngle);
      omega = (filteredAngle - lastFilteredAngle) * f / (now - lastAngleUpdateTime);
      if (abs(omega) > 2.0) {
        omega = FLT_MIN;
      } 

  
      lastAngle = angle;
      lastFilteredAngle = filteredAngle;
    }
    lastAngleUpdateTime = now;
  }
  
}

float normalizeAngle(int value) {  
  return fmap(value, 0, 1024, 0, 2.0 * PI) - fmap(ANGLE_ZERO, 0, 1024, 0, 2.0 * PI);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float smooth9(CircularBuffer<float, 9> &buf) {  
  float avg = .0;
  for (long i = 0; i < 9; i++) {
    avg += g9[i] * buf[i];
  }  
  return avg;
}

unsigned long getStepDelay(float speed) {
  direction = (speed > 0.0) ? HIGH : LOW;
  return (speed == 0) ? ULONG_MAX : (f / fabs(10000.0 * speed) - PULSE_WIDTH);
}

void runMotor() {
  unsigned long now = micros();
  if (now - lastStepTime >= stepDelay) {
    step();
    stepDelay = getStepDelay(v);   
    lastStepTime = now;  
  }
}

void step() {
  if (abs(position) > POSITION_LIMIT) {
    return; 
  }
  
  digitalWrite(DIR_PIN, direction);
  digitalWrite(STEP_PIN, HIGH); 
  delayMicroseconds(PULSE_WIDTH); 
  digitalWrite(STEP_PIN, LOW);
  position += (direction == HIGH) ? -1: 1;
}

void onADC(int index, int pin, int value) {
  adcValue = value;
  adcReady = true;
}
