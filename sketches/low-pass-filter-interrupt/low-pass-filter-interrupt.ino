#include <float.h>
#include <AnalogScanner.h>
#include <CircularBuffer.h>

const float f = 1000000.0;
const float g9[] = { 0.02376257744, 0.06195534498, 0.1228439993, 0.185233293, 0.2124095706, 0.185233293, 0.1228439993, 0.06195534498, 0.02376257744 };

CircularBuffer<float, 9> angles;
CircularBuffer<float, 9> omegas;

AnalogScanner scanner;

volatile boolean adcReady = false;
volatile int adcValue;

unsigned long lastUpdateTs = 0;

float lastAngle = FLT_MIN;

void setup() {
  Serial.begin(115200);
  
  scanner.setCallback(A0, onADC);
  int order[] = {A0};  
  scanner.setScanOrder(1, order);
  scanner.beginScanning();
}

void loop() {
  
  if (adcReady) {    
    angles.push(adcValue - 500.0);    
    adcReady = false;
  }
    
  unsigned long now = micros();
  if (now - lastUpdateTs >= 10000) {
    float angle = smooth9(angles);
    if (lastAngle != FLT_MIN) {
      float omega = (angle - lastAngle) * f / (now - lastUpdateTs);
      omegas.push(omega);      
    }

    if (omegas.isFull()) {
      Serial.print(angle, 6);
      Serial.print("\t");
      Serial.println(smooth9(omegas), 6);
    }
   
    lastAngle = angle;
    lastUpdateTs = now;
  }
}

float smooth9(CircularBuffer<float, 9> &buf) {  
  float avg = .0;
  for (long i = 0; i < 9; i++) {
    avg += g9[i] * buf[i];
  }  
  return avg;
}

void onADC(int index, int pin, int value) {
  adcValue = value;
  adcReady = true;
}
