import matplotlib.pyplot as pp
import numpy as np

from pid import PID

times = np.linspace(0, 20, 1000)
dt = times[1] - times[0]

# Motor params
R = 4.5
L = 10
K1 = 1.0
K2 = 1.0
B = 0.1
J = 2.0

state = [[1.0, 0]]

pid = PID(1.0, dt, k_p=0.8, k_i=0.2, k_d=0.004)


def derivatives(state_):
    u = 3.0
    i_, w_ = state_
    di = (-R * i - K2 * w_ + u) / L
    dw = (K1 * i - B * w_) / J
    return [di, dw]


def integrate(state_, dt_, derivatives_func):
    k1 = derivatives_func(state_)
    k2 = derivatives_func([v + d * dt_ for v, d in zip(state_, k1)])
    return [v + (k1_ + k2_) * dt_ / 2 for v, k1_, k2_ in zip(state_, k1, k2)]


for i in range(len(times)):
    state.append(integrate(state[-1], dt, derivatives))

state = np.array(state)
current = state[:, 0]
velocity = state[:, 1]

pp.plot(times, current[:-1], label="i")
pp.plot(times, velocity[:-1], label="w")
pp.grid(True)
pp.legend()
pp.show()
