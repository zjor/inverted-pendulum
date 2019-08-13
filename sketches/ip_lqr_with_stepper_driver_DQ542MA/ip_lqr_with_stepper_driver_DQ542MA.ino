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

#define HALF_TURN_PULSES_COUNT  5000

// stepper pins
#define STEP_PIN  6
#define DIR_PIN   5

#define PULSE_WIDTH             4
#define MIN_STEP_DELAY_uS       350
#define ZERO_VELOCITY_TOLERANCE 0.000001

// steps per meter
//#define VELOCITY_KOEFFICIENT    20000.0     // 1/4 step => 800 spr; 1600 steps per 8cm
#define VELOCITY_KOEFFICIENT    10000.0     // 1/2 step => 400 spr

#define ANGLE_CTRB_LIMIT  PI/36

// 17cm limit
#define POSITION_LIMIT  VELOCITY_KOEFFICIENT  * 17 / 100

// steps per revolution
#define SPR 400

#define EVOLUTION_FREQ_HZ     100

#define aKp  200.0
#define aKd  8.5

#define Kp  4.0
#define Kd  0.05

// initial displacement
#define X0  0.0
#define P0  VELOCITY_KOEFFICIENT * X0
#define V0  0.0


const float uS = 1000000.0;  // micros per second

float a = .0;         // meters per second^2
float v = V0;         // meters per second
float x = P0 / VELOCITY_KOEFFICIENT;         // calculated position

// prevoius values for improved Euler's method
float last_a = .0;
float last_v = .0;

float angle = PI;
float lastAngle = .0;
float omega = .0;

int direction = HIGH;
long position = P0;

const unsigned long evolutionPeriodMicros = 1e6 / EVOLUTION_FREQ_HZ;

unsigned long stepDelay = 0;
unsigned long lastStepTime = 0;

unsigned long lastUpdateTime = 0;

boolean interrupted = false;
boolean isControlling = false;

volatile int encoderValue = 0;
volatile int lastEncoded = 0;

void setup() {
  Serial.begin(115200);

  pinMode(OUTPUT_A, INPUT_PULLUP);
  pinMode(OUTPUT_B, INPUT_PULLUP);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(OUTPUT_A), encoderHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(OUTPUT_B), encoderHandler, CHANGE);
  
  lastStepTime = lastUpdateTime = micros();
}

int dir = LOW;

unsigned long i = 0;

void loop() {

  if (!isControlling && fabs(angle) <= ANGLE_CTRB_LIMIT) {
    isControlling = true;
  }

  if (interrupted) {
    return;
  }
  
  unsigned long now = micros();
  if (now - lastUpdateTime >= evolutionPeriodMicros) {
    float dt = 1.0 * (now - lastUpdateTime) / uS;
    integrate(dt);
    lastUpdateTime = now;
  }

  if (!isZeroVelocity(v)) {
    runMotor();
  }

//  logState();
  
}

void logState() {
  if (i % 200 == 0) {
//    Serial.print(position);
//    Serial.print("\t");
    Serial.print(x, 8);
    Serial.print("\t");
    Serial.print(v, 8);
    Serial.print("\t");
    Serial.print(angle, 3);
    Serial.print("\t");
    Serial.println(omega, 8);
  }

  i++;
}

float getControl(float th, float omega, float x, float v) {
  if (isControlling) {
    return - (Kp * x + Kd * v + aKp * th + aKd * omega);  
  } else {
    return 0.0;
//    return - (Kp * x + Kd * v);    
  }
  
}

void integrate(float dt) {
  angle = getAngle(encoderValue);
  omega = (angle - lastAngle) / dt;
  lastAngle = angle;
  
  a = getControl(angle, omega, x, v);  
  v += (a + last_a) * dt / 2;
  
//  x += (v + last_v) * dt / 2;
  x = 1.0 * position / VELOCITY_KOEFFICIENT;
  
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

void encoderHandler() {
  int MSB = digitalRead(OUTPUT_A); //MSB = most significant bit
  int LSB = digitalRead(OUTPUT_B); //LSB = least significant bit

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

float getAngle(float pulses) {  
  if (pulses > 0) {
    return PI * (1.0 - pulses / HALF_TURN_PULSES_COUNT); 
  } else {
    return -PI * (1.0 + pulses / HALF_TURN_PULSES_COUNT);
  }
}
