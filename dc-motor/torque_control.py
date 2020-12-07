import numpy as np
import matplotlib.pyplot as pp
import scipy.integrate as integrate

times = np.linspace(0, 10, 1000)
dt = times[1] - times[0]

X0 = 1.0
V0 = 0.0

x_Kp = 5.0
v_Kp = 1.0

state = [[X0, V0]]


def integrate():
    x_, v_ = state[-1]
    u = -x_Kp * x_ - v_Kp * v_
    state.append([x_ + v_ * dt, v_ + u * dt])


for i in range(len(times)):
    integrate()

state = np.array(state)
X = state[:, 0]
V = state[:, 1]

pp.plot(times, X[:-1])
pp.plot(times, V[:-1])
pp.grid(True)
pp.show()

