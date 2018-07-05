#include <arduino.h>

#define STATE_INIT_LEFT 0
#define STATE_INIT_RIGHT 1
#define STATE_INIT_MIDDLE 2
#define STATE_IDLE 3

#define DIR_RIGHT LOW
#define DIR_LEFT HIGH

#define PIN_SWITCH_A 11
#define PIN_SWITCH_B 12

#define MIN_SPEED_DELAY 2400
#define MAX_SPEED_DELAY 350

#define SPEED 700

#define RETREAT_STEPS 200

#define DEBOUNCE 500

const int stepPin = 9; 
const int dirPin = 10; 

int position = -1;
int length = -1;

void setup() {
  setupPins();
  setupMotor();

  Serial.begin(9600);
  Serial.println("Detecting available length");

  length = determineLengthAndSetToMiddle();
  position = length / 2;
  
  Serial.print("Length: "); Serial.println(length);

  position += travel(DIR_RIGHT, length / 4);
}

void setupPins() {
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);

  pinMode(PIN_SWITCH_A, INPUT);
  pinMode(PIN_SWITCH_B, INPUT);
}

void setupMotor() {
  digitalWrite(stepPin, LOW); 
}

int getReverseDirection(int direction) {
  return direction == DIR_RIGHT ? DIR_LEFT : DIR_RIGHT;
}

int step(int direction, int speed) {
  digitalWrite(dirPin, direction);

  digitalWrite(stepPin, HIGH); 
  delayMicroseconds(speed); 
  digitalWrite(stepPin, LOW); 
  delayMicroseconds(speed);

  return (direction == DIR_LEFT) ? -1 : 1;
}

int findBoundary(int direction, int speed) {
  int steps = 0;

  while (!(digitalRead(PIN_SWITCH_A) == LOW || digitalRead(PIN_SWITCH_B) == LOW)) {
    step(direction, speed);
    steps++;
  }

  Serial.print(steps); Serial.println(" steps made to boundary. Retreating...");  

  int reversedDirection = getReverseDirection(direction);
  for (int i = 0; i < RETREAT_STEPS; i++) {    
    step(reversedDirection, speed * 2);
  }

  return steps;
}

int determineLengthAndSetToMiddle() {
  findBoundary(DIR_LEFT, SPEED);    
  int position = findBoundary(DIR_RIGHT, SPEED) - RETREAT_STEPS;
  int length = position + RETREAT_STEPS * 2;

  for (int i = 0; i < length / 2 - RETREAT_STEPS; i++) {
    step(DIR_LEFT, SPEED * 1.5);
  }

  return length;
}

int travel(int direction, int steps) {
  for (int i = 0; i < steps; i++) {
    step(direction, SPEED);
  }
  return direction == DIR_LEFT ? -steps : steps;
}

int oscilationDir = DIR_LEFT;
long oscilationSpeed = SPEED * 2;

void loop() {
  float l = 1.0 * length / 2.0;
  float k = 4.0 * (MIN_SPEED_DELAY - MAX_SPEED_DELAY) / (l * l);
  float x0 = 1.0 * length / 2.0;
  oscilationSpeed = (long)(k * (position - x0) * (position - x0) + MAX_SPEED_DELAY);
  position += step(oscilationDir, oscilationSpeed);
  
  if (abs(position - x0) >= l / 2) {
    oscilationDir = getReverseDirection(oscilationDir);
  }

  // Serial.print(oscilationDir);
  // Serial.print("\t");
  // Serial.print(k);
  // Serial.print("\t");
  // Serial.print(x0);
  // Serial.print("\t");
  // Serial.print(l);
  // Serial.print("\t");
  // Serial.print(position);
  // Serial.print("\t");
  // Serial.println(oscilationSpeed);

  // delay(50);


}
