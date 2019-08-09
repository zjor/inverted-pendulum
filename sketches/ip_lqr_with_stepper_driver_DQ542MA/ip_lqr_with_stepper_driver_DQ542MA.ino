/**
 * Inverted pendulum problem with state controller
 * Encoder: OMRON E6B2-CWZ6C
 * Stepper driver: DQ542MA
 * 
 */ 

#include <float.h>
#include <limits.h>

// encoder pins
#define OUTPUT_A  2
#define OUTPUT_B  3

// stepper pins
#define STEP_PIN  6
#define DIR_PIN   5

#define PULSE_WIDTH             4
#define MIN_STEP_DELAY_uS       400
#define ZERO_VELOCITY_TOLERANCE 0.000001

// converts step velocity to m/s
#define VELOCITY_KOEFFICIENT    20000.0     // 1/4 step => 800 spr; 1600 steps per 8cm
//#define VELOCITY_KOEFFICIENT    10000.0     // 1/2 step => 400 spr

#define POSITION_LIMIT  3200

// steps per revolution
#define SPR 800

#define EVOLUTION_FREQ_HZ     100

#define Kp  4
#define Kd  0.5

// initial displacement
#define X0  0.08
#define P0  VELOCITY_KOEFFICIENT * X0
#define V0  0.0


const float uS = 1000000.0;  // micros per second

float a = .0;         // meters per second^2
float v = V0;         // meters per second
float x = P0 / VELOCITY_KOEFFICIENT;         // calculated position

// prevoius values for improved Euler's method
float last_a = .0;
float last_v = .0;

int direction = HIGH;
long position = P0;

const unsigned long evolutionPeriodMicros = 1e6 / EVOLUTION_FREQ_HZ;

unsigned long stepDelay = 0;
unsigned long lastStepTime = 0;

unsigned long lastUpdateTime = 0;

boolean interrupted = false;

volatile int encoderValue = 0;

void setup() {
  Serial.begin(115200);

  pinMode(OUTPUT_A, INPUT_PULLUP);
  pinMode(OUTPUT_B, INPUT_PULLUP);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(OUTPUT_A), aHandler, CHANGE);
  
  lastStepTime = lastUpdateTime = micros();
}

int dir = LOW;

unsigned long i = 0;

void loop() {  

  if (interrupted) {
    return;
  }

  Serial.println(encoderValue);
  delay(50);

//  for (int i = 0; i < P0; i++) {
//    step(LOW);
//    delayMicroseconds(662);
//  }
//
//  interrupted = true;
//  Serial.println(evolutionPeriodMicros);
  
//  unsigned long now = micros();
//  if (now - lastUpdateTime >= evolutionPeriodMicros) {
//    float dt = 1.0 * (now - lastUpdateTime) / uS;
//    integrate(dt);
//    lastUpdateTime = now;
//  }
//
//  if (!isZeroVelocity(v)) {
//    runMotor();
//  }

//  logState();
  
}

void logState() {
  if (i % 10 == 0) {
    Serial.print(position);
    Serial.print("\t");
    Serial.print(x, 8);
    Serial.print("\t");
    Serial.println(v, 8);
//    Serial.print("\t");
//    Serial.println(stepDelay);
  }

  i++;
  if (i > 10000) {
    interrupted = true;
  }
}

float getControl(float x, float v) {
  return - (Kp * x + Kd * v);
}


void integrate(float dt) {
  a = getControl(x, v);
  v += (a + last_a) * dt / 2;
  
  x += (v + last_v) * dt / 2;
//  x = 1.0 * position / VELOCITY_KOEFFICIENT;
  
  stepDelay = getStepDelay(v);
  
  last_a = a;
  last_v = v;
}

boolean isZeroVelocity(float speed) {
  return fabs(speed) <= ZERO_VELOCITY_TOLERANCE;
}

unsigned long getStepDelay(float speed) {
  direction = (speed > 0.0) ? HIGH : LOW;
  
  unsigned long newDelay = 
    isZeroVelocity(speed) ? 
      ULONG_MAX : 
      (uS / fabs(VELOCITY_KOEFFICIENT * speed) - PULSE_WIDTH);
        
  return max(MIN_STEP_DELAY_uS, newDelay);
}


void runMotor() {
  unsigned long now = micros();
  if (now - lastStepTime >= stepDelay) {
    step(direction);
    lastStepTime = now;  
  }
}

void step(int dir) {
  if (abs(position) > POSITION_LIMIT) {
    return; 
  }
  
  digitalWrite(DIR_PIN, dir);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(STEP_PIN, LOW);
  
  position += (dir == HIGH) ? 1: -1;
}

void aHandler() {
  int _a = digitalRead(OUTPUT_A);
  int _b = digitalRead(OUTPUT_B);
  
  if (_a == _b) {
    encoderValue++; //CW
  } else {
    encoderValue--; //CCW
  }
}
