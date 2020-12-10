import matplotlib.pyplot as pp
import numpy as np

from pid import PID

times = np.linspace(0, 5, 3000)

# Motor params
R = 4.5
L = 6e-3
K1 = 1.0
K2 = 1.0
B = 0.1
J = 2.0

state = [[0, 0]]
u_log = [0]
i_log = []

pid = PID(target=0.0, dt=(times[1] - times[0]), k_p=1.1, k_i=1.2, k_d=0.0)
pid_w = PID(target=1.0, dt=(times[1] - times[0]), k_p=10.0, k_i=0.1, k_d=0.0)


def derivatives_inductance(state_, t_, dt_):
    i_, w_ = state_
    u = pid.get_control(i_)
    u_log.append(u)
    di = (-R * i_ - K2 * w_ + u) / L
    dw = (K1 * i_ - B * w_) / J
    print(di, dw, i_, u)
    return [di, dw]


def derivatives_no_inductance(state_, t_, dt_):
    w_ = state_[0]
    u = u_log[-1]
    i_ = u / R - K2 / R * w_
    i_log.append(i_)

    w_control = pid_w.get_control(w_)
    pid.set_target(w_control)
    control = pid.get_control(i_)
    u_log.append(control)
    dw = (K1 * i_ - B * w_) / J
    return [dw]


def integrate(state_, t_, dt_, derivatives_func):
    k1 = derivatives_func(state_, t_, dt_)
    k2 = derivatives_func([v + d * dt_ for v, d in zip(state_, k1)], t_, dt_)
    return [v + (k1_ + k2_) * dt_ / 2 for v, k1_, k2_ in zip(state_, k1, k2)]


def solve(initial_state_, times_, integrate_func, derivative_func):
    dt = times_[1] - times_[0]
    states_ = [initial_state_]
    for t in times_:
        states_.append(integrate_func(states_[-1], t, dt, derivative_func))
    return np.array(states_)


state = solve([0.0], times, integrate, derivatives_no_inductance)
# current = state[:, 0]
velocity = state[:, 0]

# pp.plot(times, current[:-1], label="i")
pp.plot(times, velocity[:-1], label="w")
pp.plot(times, u_log[::2][:-1], label="U")
pp.plot(times, i_log[::2], label="i")

pp.grid(True)
pp.legend()
pp.show()
