#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <CircularBuffer.h>

const float g9[] = { 0.02376257744, 0.06195534498, 0.1228439993, 0.185233293, 0.2124095706, 0.185233293, 0.1228439993, 0.06195534498, 0.02376257744 };

Adafruit_ADS1115 ads0(0x48);

CircularBuffer<float, 9> values;

const float cutoff = 0.5;

float lastValue;
float lastFiltered;
unsigned long lastUpdateTs = 0;

int skip = 2;

void setup() {
  ads0.begin();
  Serial.begin(115200);
  
  lastUpdateTs = millis();
}

void loop() {
  float value = float(ads0.readADC_SingleEnded(0))/10.0;
  values.push(value);

  unsigned long now = millis();
  if (now - lastUpdateTs >= 5) {
    float smoothed = smooth9(values);    
    float out = lastValue + cutoff * (smoothed - lastValue);
        
    float rawD = (smoothed - lastValue) * 1000.0 / (now - lastUpdateTs);
    float filteredD = (out - lastFiltered) * 1000.0 / (now - lastUpdateTs);

    if (skip <= 0) {
      Serial.print(rawD);
      Serial.print("\t");
      Serial.print(filteredD);
      Serial.print("\t");    
      Serial.print(value);
      Serial.print("\t");
      Serial.println(out);      
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
