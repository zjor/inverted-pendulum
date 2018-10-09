/**
 * This sketch demonstrates calculation of the derivative of an analog signal.
 * 3 last samples are taken and approximated by a 2-nd order polynom.
 * The derivative of that polynom is then taken.
 * 
 * Dependencies:
 * - CircularBuffer
 */

#include <CircularBuffer.h>

CircularBuffer<double, 3> ts;
CircularBuffer<double, 3> xs;
CircularBuffer<double, 4> avg;

unsigned long lastSampleTs = 0;
const unsigned long sampleIntervalMillis = 100000;
 
void setup() {
  Serial.begin(115200);
}

void loop() {
  int value = analogRead(A0);  
  unsigned long now = micros();  
  if (now - lastSampleTs >= sampleIntervalMillis) {       
    ts.push(double(now) / 1000000.0);
    xs.push(double(value));
    lastSampleTs = now;
    if (xs.isFull()) {
      double d1 = oldDerivative(ts[1], xs[1], ts[2], xs[2]);
      double d2 = parabolicDerivative(ts[0], xs[0], ts[1], xs[1], ts[2], xs[2], ts[2]);
      avg.push(d1);

      if (avg.isFull()) {
        Serial.print(1.0 * value / 10.0);Serial.print("\t");
        Serial.print(d1, 8);Serial.print("\t");
        Serial.print((avg[0] + avg[1] + avg[2] + avg[3]) / 4.0, 8);Serial.print("\t");
        Serial.println(d2, 8);        
      }
    }
  }
}

double oldDerivative(double x0, double y0, double x1, double y1) {  
//  Serial.print(x0);Serial.print(" - ");
//  Serial.print(y0);Serial.print(" - ");
//  Serial.print(x1);Serial.print(" - ");
//  Serial.println(y1);
  
  if (x0 == x1) {    
    return 0.0;
  } else {
    return (y1 - y0) / (x1 - x0);
  }
}

double parabolicDerivative(double x0, double y0, double x1, double y1, double x2, double y2, double x) {
//  Serial.print(x0);Serial.print(" - ");
//  Serial.print(y0);Serial.print(" - ");
//  Serial.print(x1);Serial.print(" - ");
//  Serial.print(y1);Serial.print(" - ");
//  Serial.print(x2);Serial.print(" - ");
//  Serial.println(y2);
  x2 -= x0;
  x1 -= x0;
  x -= x0;
  x0 = 0.0;  

  double det = (x0 * x0 - x0 * (x1 + x2) + x1 * x2) * (x1 - x2);
  double detA = y0 * (x1 - x2) - x0 * (y1 - y2) + (y1 * x2 - x1 * y2);
  double detB = x0 * x0 * (y1 - y2) - y0 * (x1 * x1 - x2 * x2) + (x1 * x1 * y2 - x2 * x2 * y1);
  
//  Serial.print(det);Serial.print(" - ");  
//  Serial.print(detA);Serial.print(" - ");  
//  Serial.print(detB);Serial.print(" - ");
//  Serial.println((2.0 * detA * x + detB) / det);
  
  if (det == 0.0) {
    Serial.println("det == 0.0");
    return 0.0;
  } else {
    return (2.0 * detA * x + detB) / det;    
  }
}
