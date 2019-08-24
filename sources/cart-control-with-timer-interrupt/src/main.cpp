#include <Arduino.h>
#include <limits.h>

#define CPU_F           16e6
#define PRESCALER_64    (1 << CS11) | (1 << CS10)
#define PRESCALER_1024  (1 << CS12) | (1 << CS10)
#define PRESCALER       64

#define STEP_PIN    8
#define DIR_PIN     9

#define ZERO_VELOCITY_TOLERANCE 0.000001
#define STEPS_PER_METER 10000
#define POSITION_LIMIT  3200

#define EVOLUTION_FREQ_HZ     100

// PD controller constants
#define Kp  4
#define Kd  0.5

// initial displacement
#define X0  0.08
#define P0  STEPS_PER_METER * X0
#define V0  0.0

volatile unsigned int nextTimerCount = UINT_MAX;
volatile long position = P0;
volatile int direction = LOW;

float a = .0;
float v = V0;
float x = X0;

// prevoius values for improved Euler's method
float last_a = .0;
float last_v = .0;

unsigned long lastUpdateTime = 0;

const unsigned long evolutionPeriodMicros = 1e6 / EVOLUTION_FREQ_HZ;

unsigned long getStepDelayMicros(float velocity) {
  if (fabs(velocity) <= ZERO_VELOCITY_TOLERANCE) {
    return ULONG_MAX;
  } else {
    return 1e6 / (fabs(velocity * STEPS_PER_METER));
  }
}

unsigned int getTimerCount(unsigned long delayMicros) {
  if (delayMicros == ULONG_MAX) {
    return  UINT_MAX;
  }
  return (delayMicros >> 2) - 1;
}

void setDirection(float velocity) {
  direction = (velocity > .0) ? HIGH : LOW;
  digitalWrite(DIR_PIN, direction);
}

void initTimer() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= PRESCALER_64;
  TCCR1B |= (1 << WGM12); // CTC mode using OCR1A
  TIMSK1 |= (1 << OCIE1A);  //compare with OCR1A
}

void setTimerCounter(unsigned int counter) {
  OCR1A = counter;
  TCNT1 = 0;
}

void initPins() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
}

float getControl(float x, float v) {
  return - (Kp * x + Kd * v);
}

void integrate(float dt) {
  a = getControl(x, v);
  v += (a + last_a) * dt / 2;
  
  // x += (v + last_v) * dt / 2;
  x = 1.0 * position / STEPS_PER_METER;
  setDirection(v);
  nextTimerCount = getTimerCount(getStepDelayMicros(v));

  OCR1A = nextTimerCount;
  if (TCNT1 > OCR1A) {
    TCNT1 = 0;
  }
  
  last_a = a;
  last_v = v;
}


unsigned long lastLogTime = 0;

void logState() {
  unsigned long now = micros();
  if (now - lastLogTime >= 250000) {
    Serial.print(a, 4);
    Serial.print("\t");
    Serial.print(v, 4);
    Serial.print("\t");
    Serial.print(x, 4);
    Serial.print("\t");
    Serial.println(nextTimerCount);
    lastLogTime = now;
  }
}

void setup() {  
  initTimer();
  initPins();
  setTimerCounter(2000);
  setDirection(.0);
  
  lastUpdateTime = micros();
  Serial.begin(9600);
  sei();
}

void loop() {
  unsigned long now = micros();
  if (now - lastUpdateTime >= evolutionPeriodMicros) {
    float dt = 1.0 * (now - lastUpdateTime) / 1e6;
    integrate(dt);
    lastUpdateTime = now;
  }

  logState();
}

ISR (TIMER1_COMPA_vect) {

  if (nextTimerCount == UINT_MAX) {
    return;
  }

  if (abs(position) >= POSITION_LIMIT) {
    return;
  }

  OCR1A = nextTimerCount;
  
  digitalWrite(STEP_PIN, HIGH);
  digitalWrite(STEP_PIN, LOW);

  if (direction == HIGH) {
    position++;
  } else {
    position--;
  }
  
}