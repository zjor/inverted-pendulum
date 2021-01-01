import matplotlib.pyplot as pp
import numpy as np

from pid import PID
from ode_solver import solve, integrate_rk4

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
def derivatives_inductance(state_, step, t_, dt_):
    i_, w_ = state_
    u = pid.get_control(i_)
    u_log.append(u)
    di = (-R * i_ - K2 * w_ + u) / L
    dw = (K1 * i_ - B * w_) / J
    print(di, dw, i_, u)
    return [di, dw]


def derivatives_no_inductance(state_, step, t_, dt_):
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


state = solve([0.0, 0.0], times, integrate_rk4, derivatives_no_inductance)
position = state[:, 0]
velocity = state[:, 1]

pp.plot(times, position[:-1], label="x")
pp.plot(times, velocity[:-1], label="w")
pp.plot(times, u_log[::4][:-1], label="U")
pp.plot(times, i_log[::4], label="i")

pp.grid(True)
pp.legend()
pp.show()
