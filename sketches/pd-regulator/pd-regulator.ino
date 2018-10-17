/**
 * This sketch demonstrates how to bring a cart to [x: 0; v: 0] from non-zero position using PD-regulator
 */
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <CircularBuffer.h>
 
#define STEP_PIN    9
#define DIR_PIN     10
#define PULSE_WIDTH 4

#define POSITION_LIMIT  1500

#define ADS0_MIN  242
#define ADS0_MAX  24141
#define ANGLE_ZERO  3.24

// -26.22131018  -7.50023911  -0.4472136   -1.30242157

#define Kp  0.447
#define Kd  1.302

#define aKp  26.221
#define aKd  7.5

const float g9[] = {0.02376257744, 0.06195534498, 0.1228439993, 0.185233293, 0.2124095706, 0.185233293, 0.1228439993, 0.06195534498, 0.02376257744 };
const float cutoff = 0.5;

CircularBuffer<float, 9> angles;
Adafruit_ADS1115 ads0(0x48);  //angle sensor ADC

float a = .0; // meters per second^2
float v = .0; //meters per second
float f = 1000000.0; //micros per second

int direction = HIGH;
long position = 0;

unsigned long stepDelay = 0;
unsigned long lastStepTime = 0;

unsigned long lastEvolutionTime = 0;
unsigned long evolutionPeriod = f / 100;

unsigned long lastAngleUpdateTime = 0;
unsigned long angleUpdatePeriod = f / 200;
float lastAngle;
float filteredAngle;
float lastFilteredAngle;
float omega;

void setup() {  
  pinMode(STEP_PIN, OUTPUT); 
  pinMode(DIR_PIN, OUTPUT);
  ads0.begin();
//  Serial.begin(115200);
}

long i = 0;

void loop() {
  updateAngleAndDerivative();
  evolveWorld();
  runMotor();

//  if (i % 10 == 0) {
//    Serial.print(filteredAngle, 6);
//    Serial.print("\t");
//    Serial.print(omega, 6);
//    Serial.print("\t");
//    Serial.print(position);
//    Serial.print("\t");
//    Serial.println(v, 6);   
//  }
//  i++;
  
}

float getControl(float th, float omega, float x, float v) {
  return - (Kp * x + Kd * v + aKp * th + aKd * omega);
}

void evolveWorld() {
  unsigned long now = micros();
  if (now - lastEvolutionTime >= evolutionPeriod) {
    float a = getControl(filteredAngle, omega, float(position) / 10000.0, v);
    
    float dt = float(now - lastEvolutionTime) / f;
    v += a * dt;
    stepDelay = getStepDelay(v);
    lastEvolutionTime = now;
  }
}

//state variables: filteredAngle, omega
void updateAngleAndDerivative() {
  angles.push(normalizeAngle(ads0.readADC_SingleEnded(0)));
  unsigned long now = micros();
  if (now - lastAngleUpdateTime >= angleUpdatePeriod) {
    float angle = smooth9(angles);
    filteredAngle = lastAngle + cutoff * (angle - lastAngle);
    omega = (filteredAngle - lastFilteredAngle) * f / (now - lastAngleUpdateTime);

    lastAngle = angle;
    lastFilteredAngle = filteredAngle;
    lastAngleUpdateTime = now;
  }
  
}

float normalizeAngle(int16_t value) {  
  return fmap(value, ADS0_MIN, ADS0_MAX, 0, 2.0 * PI) - ANGLE_ZERO;
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
  return (speed == 0) ? 0 : (f / fabs(10000.0 * speed) - PULSE_WIDTH);
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
  position += (direction == HIGH) ? 1 : -1;
}
