# Cart position control with timer interrupt

This code controls position of a cart with a stepper motor. Timer interrupt is used to control step delay precisely.

## Hardware
- DQ542MA stepper driver
- Arduino Uno
- Nema 17 stepper motor: 17HS4401

## Thoughts

Usage of interrupts allows logging system's state to console.
The problem arises when the motor should be driven very slowly, velocity update happens each step, e.i. each ISR call and if there is enough time between calls the system will evolve before the change is recognised and reflected in motion.

## References
- (http://maxembedded.com/2011/07/avr-timers-ctc-mode/)
- (https://www.instructables.com/id/Arduino-Timer-Interrupts/)