import matplotlib.pyplot as pp
import numpy as np

from pid import PID

times = np.linspace(0, 15, 3000)
dt = times[1] - times[0]

# Motor params
R = 4.5
L = 6e-3
K1 = 1.0
K2 = 1.0
B = 0.1
J = 2.0

u_log = [0]
i_log = []

pid = PID(target=0.0, dt=dt, k_p=1.5, k_i=1.2, k_d=0.003, min_value=-12, max_value=12)
pid_w = PID(target=0.0, dt=dt, k_p=4.0, k_i=0.2, k_d=0.01)
pid_x = PID(target=2.0, dt=dt, k_p=5.0, k_i=0.2, k_d=0.01)


# deprecated as we neglect the inductance L
def derivatives_inductance(state_, t_, dt_):
    i_, w_ = state_
    u = pid.get_control(i_)
    u_log.append(u)
    di = (-R * i_ - K2 * w_ + u) / L
    dw = (K1 * i_ - B * w_) / J
    print(di, dw, i_, u)
    return [di, dw]


def derivatives_no_inductance(state_, t_, dt_):
    x_, w_ = state_
    u = u_log[-1]
    i_ = u / R - K2 / R * w_
    i_log.append(i_)
    x_control = pid_x.get_control(x_)
    pid_w.set_target(x_control)

    w_control = pid_w.get_control(w_)
    pid.set_target(w_control)

    i_control = pid.get_control(i_)

    u_log.append(i_control)
    dw = (K1 * i_ - B * w_) / J
    dx = w_
    return [dx, dw]


def integrate(state_, t_, dt_, derivatives_func):
    k1 = derivatives_func(state_, t_, dt_)
    k2 = derivatives_func([v + d * dt_ for v, d in zip(state_, k1)], t_, dt_)
    return [v + (k1_ + k2_) * dt_ / 2 for v, k1_, k2_ in zip(state_, k1, k2)]


def solve(initial_state_, times_, integrate_func, derivative_func):
    dt_ = times_[1] - times_[0]
    states_ = [initial_state_]
    for t in times_:
        states_.append(integrate_func(states_[-1], t, dt_, derivative_func))
    return np.array(states_)


state = solve([0.0, 0.0], times, integrate, derivatives_no_inductance)
position = state[:, 0]
velocity = state[:, 1]

pp.plot(times, position[:-1], label="x")
pp.plot(times, velocity[:-1], label="w")
pp.plot(times, u_log[::2][:-1], label="U")
pp.plot(times, i_log[::2], label="i")

pp.grid(True)
pp.legend()
pp.show()
