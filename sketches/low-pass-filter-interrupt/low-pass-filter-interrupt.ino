#include <float.h>

#define M 10
#define N 9

const float f = 1000000.0;

const byte adcPin = 0;
volatile int adcReading;
volatile boolean adcDone;
boolean adcStarted;
int value;

int values[M];
int ix = 0;

float ds[N];
int dIx;

float lastValue = FLT_MIN;

unsigned long samplesCount = 0;
unsigned long lastTimestamp = 0;
unsigned long angleInterval = 5000;

void setup () {
  Serial.begin (115200);
  ADCSRA =  bit (ADEN);                     
  ADCSRA |= bit (ADPS2);
  ADMUX  =  bit (REFS0) | (adcPin & 0x07);
}  

ISR (ADC_vect) {
  adcReading = ADC;
  adcDone = true;  
}  
  
void loop () {
  if (adcDone) {
    adcStarted = false;
    value = adcReading;
    samplesCount++;
    adcDone = false;
  }
    
  if (!adcStarted) {
    adcStarted = true;
    ADCSRA |= bit (ADSC) | bit (ADIE);
  }

  values[ix++] = value;
  ix = (ix >= M) ? 0 : ix;  

  unsigned long now = millis();
  if (now - lastTimestamp >= 10) {
    float sVal = smooth(values);
    float dt = (float) 10.0 / 1000.0;
    float d = (sVal - lastValue) / dt;
    
    ds[dIx++] = d;
    dIx = (dIx >= N) ? 0 : dIx;
    
    if (lastValue != FLT_MIN && fabs(d) < 5000.0) {
      Serial.print(samplesCount);
      Serial.print("\t");
      Serial.print(sVal);
      Serial.print("\t");
      Serial.println(fsmooth(ds), 6);
    }
    samplesCount = 0;
    lastValue = sVal;
    lastTimestamp = now;
  }
  
}

const float g9[] = { 0.02376257744, 0.06195534498, 0.1228439993, 0.185233293, 0.2124095706, 0.185233293, 0.1228439993, 0.06195534498, 0.02376257744 };

float smooth(int values[]) {
  float val = .0;
  for (int i = 0; i < M; i++) {
    val += values[i];
  }
  return val / M;
}

float fsmooth(float values[]) {
  float val = .0;
  for (int i = 0; i < N; i++) {
    val += ((float)values[i]) * g9[i];
  }
  return val;
}
