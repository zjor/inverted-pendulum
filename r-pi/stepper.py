import time
import platform
from threading import Lock

MICROSECONDS = 1000000.0

if platform.system() == 'Linux':
    import RPi.GPIO as GPIO

    class Stepper:

        # Direction constants 
        CW = 1
        CCW = -1

        def __init__(self, dir_pin, step_pin, lock, initial_position=0, pulse_width_micros=4):            
            self.dir_pin = dir_pin
            self.step_pin = step_pin
            self.lock = lock
            self.pulse_width = float(pulse_width_micros) / MICROSECONDS
            self.position = initial_position
            self.init_gpio()


        def step(self, direction):
            GPIO.output(self.dir_pin, GPIO.HIGH if direction == Stepper.CW else GPIO.LOW)
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(self.pulse_width)
            GPIO.output(self.step_pin, GPIO.LOW)
            with self.lock:
                self.position += direction

        def get_position(self):
            with self.lock:
                return self.position


        def init_gpio(self):
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.step_pin, GPIO.OUT)
            GPIO.setup(self.dir_pin, GPIO.OUT)        

else:

    class Stepper:

        # Direction constants 
        CW = 1
        CCW = -1

        def __init__(self, dir_pin, step_pin, lock, initial_position=0, pulse_width_micros=4):
            self.dir_pin = dir_pin
            self.step_pin = step_pin
            self.lock = lock
            self.pulse_width = float(pulse_width_micros) / MICROSECONDS
            self.position = initial_position


        def step(self, direction):
            time.sleep(self.pulse_width)
            with self.lock:
                self.position += direction

        def get_position(self):
            with self.lock:
                return self.position


if __name__ == "__main__":
    stepper = Stepper(13, 11, Lock())
    for i in range(100):
        stepper.step(Stepper.CW)
        time.sleep(0.003)

    time.sleep(0.5)

    for i in range(100):
        stepper.step(Stepper.CCW)
        time.sleep(0.002)

    GPIO.cleanup()        

    print("Position: %d" % stepper.get_position())

