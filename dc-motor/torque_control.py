import numpy as np
import matplotlib.pyplot as pp
import scipy.integrate as integrate

times = np.linspace(0, 10, 1000)

X0 = 1.0
V0 = 0.0

x_Kp = 5.0
v_Kp = 1.0

state = np.array([X0, V0])


def derivatives(state_, t):
    x, v = state_
    u = -x_Kp * x - v_Kp * v
    ds = np.zeros_like(state_)
    ds[0] = v
    ds[1] = u
    return ds


solution = integrate.odeint(derivatives, state, times)

pp.plot(times, solution[:, 0], label="X")
pp.plot(times, solution[:, 1], label="V")
pp.legend()
pp.grid(True)
pp.show()
