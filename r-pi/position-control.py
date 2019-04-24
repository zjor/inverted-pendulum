import time
import threading
import platform
from stepper import Stepper

MICROSECONDS = 1000000.0
STEPS_PER_10CM = 10000.0

Kp = 4.0
Kd = 3.0


class World:

    def __init__(self, x, target_x, v, lock, pulse_width):
        self.x = x
        self.target_x = target_x
        self.v = v
        self.lock = lock
        self.pulse_width = pulse_width
        self.step_delay = 0


    def get_control(self, x, target_x, v):
        return -(Kp * (x - target_x) + Kd * v)


    def get_step_delay(self, v):
        if abs(v) < 0.0000001:
            return 0
        else:
            return 1.0 / (STEPS_PER_10CM * abs(v)) - self.pulse_width / MICROSECONDS


    def evolve(self, observed_x, dt):
        with self.lock:
            a = self.get_control(observed_x, self.target_x, self.v)
            self.v += a * dt
            self.x += self.v * dt
            self.step_delay = self.get_step_delay(self.v)


    def get_state(self):
        with self.lock:
            return (self.x, self.v)


def run_motor(stepper, world, stopped):
    last_step_time = time.time()
    while not stopped.isSet():
        with world.lock:
            step_delay = world.step_delay
            v = world.v
        now = time.time()
        if step_delay > 0 and now - last_step_time >= step_delay:
            stepper.step(Stepper.CW if v > 0 else Stepper.CCW)
            last_step_time = now
        time.sleep(step_delay / 7)


def update_world(world, stepper, update_timeout, stopped):
    last_update = time.time()
    while not stopped.isSet():
        now = time.time()
        dt = now - last_update
        if dt >= update_timeout:
            world.evolve(stepper.get_position() / STEPS_PER_10CM, dt)
            last_update = now
        time.sleep(update_timeout / 7)


if __name__ == "__main__":
    lock = threading.Lock()

    stepper = Stepper(13, 11, lock)
    world = World(.0, .1, .0, lock, stepper.pulse_width)
    stopped = threading.Event()

    motor_thread = threading.Thread(name='motor', target=run_motor, args=(stepper, world, stopped))
    world_thread = threading.Thread(name='world', target=update_world, args=(world, stepper, 0.005, stopped))

    print("Press CTRL+C to interrupt")
    try:
        motor_thread.start()
        world_thread.start()

        while True:
            state = world.get_state()
            position = stepper.get_position()

            print("V = %f;\tEstimated X = %f;\tReal X = %f" % (state[1], state[0], float(position) / STEPS_PER_10CM))
            time.sleep(0.1)

    except KeyboardInterrupt:
        stopped.set()
        if platform.system() == 'Linux':
            import RPi.GPIO as GPIO
            GPIO.cleanup()

