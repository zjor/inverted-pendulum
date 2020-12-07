import matplotlib.pyplot as pp
import numpy as np

times = np.linspace(0, 20, 1000)
dt = times[1] - times[0]

# Motor params
R = 4.5
L = 10
K1 = 1.0
K2 = 1.0
B = 0.1
J = 2.0

state = [[0, 0]]


class PID:
    def __init__(self, target, dt, k_p=0.5, k_i=0.0, k_d=0):
        self.target = target
        self.dt = dt
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.last_value = None
        self.err_acc = 0.0

    def get_control(self, value):
        e = self.target - value
        de = - ((value - self.last_value) / dt) if self.last_value else 0.0
        self.err_acc += self.k_i * e * dt
        self.last_value = e
        return self.k_p * e + self.err_acc + self.k_d * de


pid = PID(1.0, dt, k_p=0.8, k_i=0.2, k_d=0.004)


def derivatives(state_, u):
    i_, w_ = state_
    di = (-R * i - K2 * w_ + u) / L
    dw = (K1 * i - B * w_) / J
    return [di, dw]


def integrate():
    i_, w_ = state[-1]
    di, dw = derivatives(state[-1], 3.0)
    print(di, dw)
    state.append([i_ + di * dt, w_ + dw * dt])


for i in range(len(times)):
    integrate()

state = np.array(state)
current = state[:, 0]
velocity = state[:, 1]

pp.plot(times, current[:-1], label="i")
pp.plot(times, velocity[:-1], label="w")
pp.grid(True)
pp.legend()
pp.show()
