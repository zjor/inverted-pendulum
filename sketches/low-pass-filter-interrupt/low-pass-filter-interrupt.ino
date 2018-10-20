#include <AnalogScanner.h>
#include <CircularBuffer.h>

const float f = 1000000.0;
const float g9[] = { 0.02376257744, 0.06195534498, 0.1228439993, 0.185233293, 0.2124095706, 0.185233293, 0.1228439993, 0.06195534498, 0.02376257744 };

CircularBuffer<float, 9> values;

const float cutoff = 0.5;

AnalogScanner scanner;

volatile boolean adcReady = false;
volatile int adcValue;

float value;
float lastValue;
float lastFiltered;
unsigned long lastUpdateTs = 0;

int skip = 9;
unsigned long t = 0;

void setup() {
  Serial.begin(115200);
  
  scanner.setCallback(A0, onADC);
  int order[] = {A0};  
  scanner.setScanOrder(1, order);
  scanner.beginScanning();
}

void loop() {
  
  if (adcReady) {
    
    value = adcValue;
    values.push(value);
    
    adcReady = false;
  }
  
  
  unsigned long now = micros();
  if (now - lastUpdateTs >= 100000) {
    float smoothed = smooth9(values);    
    float out = lastValue + cutoff * (smoothed - lastValue);
        
    float rawD = (smoothed - lastValue) * f / (now - lastUpdateTs);
    float filteredD = (out - lastFiltered) * f / (now - lastUpdateTs);

    if (skip <= 0) {
      Serial.print(rawD);
      Serial.print("\t");
      Serial.println(filteredD);
//      Serial.print("\t");    
//      Serial.print(value);
//      Serial.print("\t");
//      Serial.println(out);      
    } else {
      skip--;
    }
    
    
    lastFiltered = out;
    lastValue = smoothed;
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
