import time
import threading

from math import pi
from numpy import sin, cos

class Pendulum:

    G = 9.81

    def __init__(self, l, th0, w0, cart):        
        self.lock = threading.RLock()
        self.l = l
        self.th = th0
        self.w = w0
        self.x = []
        self.cart = cart


    def integrate(self, dt):
        c_state = cart.get_state()

        with self.lock:
            dw = (Pendulum.G * sin(self.th) - c_state[2] * cos(self.th)) / self.l
            self.w += dw * dt
            self.th += self.w * dt
            return self.get_state()


    def get_state(self):
        with self.lock:
            return (self.th, self.w)


    def set_cart_position(self, x):
        with self.lock:
            self.x.append(x)
            self.x = self.x[-3:]    # last 3 elements


class Cart:

    def __init__(self):
        self.lock = threading.RLock()
        self.x = .0
        self.v = .0
        self.u = .0


    def set_control(self, u):
        with self.lock:
            self.u = u


    def get_state(self):
        with self.lock:
            return (self.x, self.v, self.u)


    def integrate(self, dt):
        with self.lock:
            self.v += self.u * dt
            self.x += self.v * dt
            return self.get_state()



def pendulum_update(pendulum, cart, freq_kHz, stoppped):
    dt = 1.0 / (freq_kHz * 1000)
    while not stoppped.isSet():
        c_state = cart.integrate(dt)
        pendulum.set_cart_position(c_state[0])
        pendulum.integrate(dt)        
        time.sleep(dt)


pKp = 50.0
pKd = 15.0
cKp = 3.0
cKd = 4.0


def control_cart(pendulum, cart, freq_kHz, stoppped):
    dt = 1.0 / (freq_kHz * 1000)
    while not stoppped.isSet():
        c_state = cart.get_state()
        p_state = pendulum.get_state()
        u = (pKp * p_state[0] + pKd * p_state[1] + cKp * c_state[0] + cKd * c_state[1])
        cart.set_control(u)
        time.sleep(dt)


def logger(pendulum, cart, freq_kHz, stoppped):
    dt = 1.0 / (freq_kHz * 1000)
    while not stoppped.isSet():
        now = time.time()
        c_state = cart.get_state()
        p_state = pendulum.get_state()

        # x, v, th, w
        print "%f\t%f\t%f\t%f\t%f" % (now, c_state[0], c_state[1], p_state[0], p_state[1])
        time.sleep(dt)



if __name__ == "__main__":
    import numpy as np
    import matplotlib.pyplot as pp
    import matplotlib.animation as animation

    dt = 0.01

    cart = Cart()
    pendulum = Pendulum(1.0, .03, .0, cart)
    

    stopped = threading.Event()
    pendulum_thread = threading.Thread(name="pendulum", target=pendulum_update, args=(pendulum, cart, 2, stopped))
    control_thread = threading.Thread(name="control", target=control_cart, args=(pendulum, cart, 0.1, stopped))
    logger_thread = threading.Thread(name="logger", target=logger, args=(pendulum, cart, 0.01, stopped))

    pendulum_thread.start()
    control_thread.start()
    logger_thread.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        stopped.set()
        


