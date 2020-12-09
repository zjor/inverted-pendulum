import matplotlib.pyplot as pp
import numpy as np

from pid import PID

times = np.linspace(0, 10, 1000)
dt = times[1] - times[0]

# Motor params
R = 4.5
L = 6e-3
K1 = 1.0
K2 = 1.0
B = 0.1
J = 2.0

state = [[0, 0]]
u_log = []

pid = PID(target=1.0, dt=dt, k_p=0.02, k_i=0.0, k_d=0.0)


# TODO: neglect L
def derivatives(state_, t_, dt_):
    i_, w_ = state_
    u = pid.get_control(i_)
    u_log.append(u)
    di = (-R * i_ - K2 * w_ + u) / L
    dw = (K1 * i_ - B * w_) / J
    print(di, dw, i_, u)
    return [di, dw]


# TODO: test with simulation of 3-body problem
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
pp.plot(times, u_log[::2], label="U")
pp.grid(True)
pp.legend()
pp.show()
