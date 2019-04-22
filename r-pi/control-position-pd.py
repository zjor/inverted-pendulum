import time
import multiprocessing as mp

import Adafruit_ADS1x15
import RPi.GPIO as GPIO

from math import pi

# Pins
stepPin = 11
dirPin = 13

# Constants
U_SECONDS = 1000000.0
PULSE_WIDTH = 4.0

# PD params
Kp = 3.0
Kd = 2.0

# Initial conditions
v0 = 0.0
x0 = 0.0


def init_gpio():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(stepPin, GPIO.OUT)
    GPIO.setup(dirPin, GPIO.OUT)


def step(direction):
    GPIO.output(dirPin, direction)
    GPIO.output(stepPin, GPIO.HIGH)
    time.sleep(PULSE_WIDTH / U_SECONDS)
    GPIO.output(stepPin, GPIO.LOW)


def run_motor(direction, step_delay, position, started):
    while started.value == 1:
        if step_delay.value > 0:
            step(direction.value)
            time.sleep(step_delay.value)
            position.value += 1 if direction.value == GPIO.HIGH else -1


def get_control(x, x0, v):
    return -(Kp * (x - x0) + Kd * v)


def get_step_delay(v):
    if abs(v) < 0.000001:
        return 0
    else:
        return 1.0 / (10000.0 * abs(v)) - PULSE_WIDTH / U_SECONDS


def integrate(x, x0, v, last_update, update_timeout, direction, position, step_delay, started):
    while started.value == 1:
        now = time.time()
        dt = now - last_update.value
        if dt >= update_timeout:
            a = get_control(position.value / 10000.0, x0, v.value)
            v.value += a * dt
            x.value += v.value * dt
            direction.value = GPIO.HIGH if v.value > .0 else GPIO.LOW
            step_delay.value = get_step_delay(v.value)
            last_update.value = now
        time.sleep(update_timeout)


if __name__ == "__main__":
    init_gpio()

    x = mp.Value('d', 0.1)
    v = mp.Value('d', v0)
    last_update = mp.Value('d', time.time())
    direction = mp.Value('i', GPIO.HIGH)
    step_delay = mp.Value('d', .0)
    started = mp.Value('i', 1)
    position = mp.Value('i', int(x.value * 10000))

    integrate_process = mp.Process(target=integrate, args=(x, x0, v, last_update, 0.001, direction, position, step_delay, started))
    motor_process = mp.Process(target=run_motor, args=(direction, step_delay, position, started))

    motor_process.start()
    integrate_process.start()

    print("Press CTRL+C to exit")
    try:
        while 1:
            print "v:\t%f;\tx:\t%f;\treal x:\t%f" % (v.value, x.value, position.value / 10000.0)
            time.sleep(0.5)
    except KeyboardInterrupt:
        GPIO.cleanup()
        started.value = 0
        integrate_process.join()
        motor_process.join()
