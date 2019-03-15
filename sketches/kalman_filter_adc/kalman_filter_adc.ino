#define ANGLE_REF 505

boolean adcStarted;

volatile int adcValue;
volatile boolean adcDone;

unsigned long timestamp;
const unsigned long UPDATE_TIMEOUT_MICROS = 5000;

int lastValue = -1;

void adc_init() {
    ADMUX  = _BV(REFS0)    // ref = AVCC
           | 0;         // input channel
    ADCSRB = 0;          // free running mode
    ADCSRA = _BV(ADEN)   // enable
           | _BV(ADSC)   // start conversion
           | _BV(ADATE)  // auto trigger enable
           | _BV(ADIF)   // clear interrupt flag
           | _BV(ADIE)   // interrupt enable
           | 7;          // prescaler = 128
}

void setup() {
  adc_init();
  Serial.begin(115200);
  timestamp = micros();
}

const float k = 10.0;
const float g = 9.81;
const float L = 0.431;

float th = .0;
float Y = .0;
float lastTh = .0;

void loop() {
  if (adcDone) {
    adcDone = false;
    unsigned long now = micros();
    
    if (now - timestamp > UPDATE_TIMEOUT_MICROS) {
      float dt = 1.0 * (now - timestamp) / 1000000.0;
      float y = normalizeAngle(adcValue, ANGLE_REF);

      float observedY = (y - lastTh) / dt;
                  
      float dTh = Y + k * (y - th);
      float dY = - g / L * th;
      
      Y = Y + dY * dt;
      th = th + dTh * dt;

      lastTh = y;
      timestamp = now;

      Serial.print(y, 6);
      Serial.print("\t");
      Serial.print(th, 6);
      Serial.print("\t");
      Serial.print(Y, 6);
      Serial.print("\t");
      Serial.println(observedY, 6);
    }
  }

}

float normalizeAngle(int value, int ref) {  
  return fmap(value - ref, 0, 1024, 0, 2.0 * PI);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

ISR (ADC_vect) {
  adcValue = ADC;
  adcDone = true;  
}
