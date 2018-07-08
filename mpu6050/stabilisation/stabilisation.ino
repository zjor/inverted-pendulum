#include <math.h>
#include <limits.h>

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* 
Connecting to Arduino UNO:
  SDA -> A4
  SCL -> A5
  INT -> D2
*/

#define OUTPUT_READABLE_YAWPITCHROLL


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

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

unsigned long nowMillis = 0;

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

void waitForAnyCharacter() {
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
}

void initCartLocation() {
  Serial.println("Determining rail center...");
  length = determineLengthAndSetToMiddle();
  position = length / 2;
  
  Serial.print("Length: "); Serial.println(length);
}

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    setupPins();
    setupMotor();

    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("\nSend any character to start"));
    waitForAnyCharacter();

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    initCartLocation();

    Serial.println(F("\nSend any character to continue"));
    waitForAnyCharacter();
    nowMillis = millis();
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

int step(int direction, int speed) {
  digitalWrite(dirPin, direction);

  digitalWrite(stepPin, HIGH); 
  delayMicroseconds(speed); 
  digitalWrite(stepPin, LOW); 
  delayMicroseconds(speed);

  return (direction == DIR_LEFT) ? -1 : 1;
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

#define ANGLE_NOT_SET -1000.0

double angle = ANGLE_NOT_SET;
double lastAngle = ANGLE_NOT_SET;
bool halted = false;

#define LOOP_STATE_INIT           1
#define LOOP_STATE_DELAY1         2
#define LOOP_STATE_STEP_CONTINUE  3
#define LOOP_STATE_DELAY2         4
#define LOOP_STATE_CONTINUE       5

int loopState = LOOP_STATE_INIT;

unsigned long delayStartMicros = 0;

unsigned long lastStepDelay = ULONG_MAX;
unsigned long stepDelay = ULONG_MAX;
double velocity = 0.0;
double accel = 0.0;

int dir = DIR_LEFT;

const double r = 0.0135;
const double k = M_PI * r / 100;

const double Kp = 20; //45.0;
const double Kd = 18.0;

const double xKp = 2.0;
const double xKd = 3.0;

double getControl(double a, double da, double dx, double v) {
  return - (Kp * a + Kd * da + xKp * dx * k + xKd * v);
}

void moveCartLoop() {
  double dt;  
  double da;  
  double dx;  
  
  if (!halted) {
    switch (loopState) {
      case LOOP_STATE_INIT:
        if (angle == ANGLE_NOT_SET) {
          break;
        }
        if (lastAngle == ANGLE_NOT_SET) {
          lastAngle = angle;
        }
        if (millis() - nowMillis < 10) {
          // too soon
          break;
        }
        dt = 1.0 * (millis() - nowMillis)/1000;
        da = (angle - lastAngle) / dt;
        dx = length / 2 - position;

        Serial.print("dt: ");Serial.print(dt, 4);
        Serial.print("; a: ");Serial.print(angle, 4);
        Serial.print("; da: ");Serial.print(da, 4);
        Serial.print("; dx: ");Serial.print(dx, 4); 
        Serial.print("; v: ");Serial.println(velocity, 4);
        
        accel = getControl(angle, da, dx, velocity);
        velocity += accel * dt;
        lastAngle = angle;
        if (abs(velocity) > 0.01) {
          stepDelay = 100.0 * k / abs(velocity) + 200;
        }
        dir = velocity < 0 ? DIR_LEFT : DIR_RIGHT;

        Serial.print("Accel: ");Serial.print(accel);Serial.print("; velocity: ");Serial.print(velocity);Serial.print("; step: ");Serial.println(stepDelay);
                
        if (abs(velocity) > 0.01) {
          digitalWrite(dirPin, dir);        
          digitalWrite(stepPin, HIGH); 
          loopState = LOOP_STATE_DELAY1;
        }
        delayStartMicros = micros();
        nowMillis = millis();
        break;
      case LOOP_STATE_DELAY1:
        if (micros() - delayStartMicros >= stepDelay) {
          loopState = LOOP_STATE_STEP_CONTINUE;          
        }
        break;
      case LOOP_STATE_STEP_CONTINUE:
        digitalWrite(stepPin, LOW);
        loopState = LOOP_STATE_DELAY2;
        delayStartMicros = micros();
        break;
      case LOOP_STATE_DELAY2:
        if (micros() - delayStartMicros >= stepDelay) {
          loopState = LOOP_STATE_CONTINUE;
        }
        break;
      case LOOP_STATE_CONTINUE:
        Serial.print("Stepping: ");
        Serial.println(dir == DIR_LEFT ? stepDelay : -((long)stepDelay));
        position += (dir == DIR_LEFT) ? -1 : 1;
        loopState = LOOP_STATE_INIT;
        break;
    }
  }
  if (digitalRead(PIN_SWITCH_A) == LOW || digitalRead(PIN_SWITCH_B) == LOW) {
    halted = true;
  }  
}

void loop() {
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      moveCartLoop();
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);
            angle = ypr[2];            
        #endif        
    }
}
