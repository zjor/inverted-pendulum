import matplotlib.pyplot as pp
import numpy as np

from pid import PID

times = np.linspace(0, 0.1, 100)
dt = times[1] - times[0]

# Motor params
R = 4.5
L = 6e-3
K1 = 1.0
K2 = 1.0
B = 0.1
J = 2.0

state = [[0, 0]]

pid = PID(1.0, dt, k_p=0.8, k_i=0.2, k_d=0.004)


def derivatives(state_, t_, dt_):
    u = 4.0
    i_, w_ = state_
    di = (-R * i_ - K2 * w_ + u) / L
    dw = (K1 * i_ - B * w_) / J
    print(di, dw, i_)
    return [di, dw]


def integrate(state_, t_, dt_, derivatives_func):
    k1 = derivatives_func(state_, t_, dt_)
    k2 = derivatives_func([v + d * dt_ for v, d in zip(state_, k1)], t_, dt_)
    return [v + (k1_ + k2_) * dt_ / 2 for v, k1_, k2_ in zip(state_, k1, k2)]


for t in times:
    state.append(integrate(state[-1], t, dt, derivatives))

state = np.array(state)
current = state[:, 0]
velocity = state[:, 1]

pp.plot(times, current[:-1], label="i")
pp.plot(times, velocity[:-1], label="w")
pp.grid(True)
pp.legend()
pp.show()
