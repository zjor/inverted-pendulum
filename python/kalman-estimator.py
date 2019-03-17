'''
Dynamics:
	th'' = - g / L * sin(th) - d * th', where d - rotation friction coefficient

Linearisation:
	th' = Y
	Y' = - g / L * th - d * Y

Estimation:
	y = trim(th)
	x_h' = A * x_h + K * (y - Y_h)	
	Y_h = - g / L * th_h - d * Y_h
'''

import numpy as np
import matplotlib.pyplot as pp
import scipy.integrate as integrate
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

from math import pi, trunc
from numpy import sin, cos

def trim(x, step):
    d = trunc(x / step)
    return step * d

# physical constants
g = 9.8
L = 1.0
d = 0.05	# friction coefficient [1/sec]

# simulation time
dt = 0.05
Tmax = 10
t = np.arange(0.0, Tmax, dt)

# initial conditions
Y0 = .0 	# pendulum angular velocity
th0 = pi/12	# pendulum angle
precision = 0.006

# th = [th0]
# Y = [Y0]
# th_h = [trim(th0, precision)]
# Y_h = [.0]
k = 100.0

state = np.array([th0, Y0, trim(th0, precision), Y0])

def disturbance(t, u):
	if t >= 1.5 and t < 2:
		return u
	elif t >= 2 and t < 2.5:
		return - u
	else:
		return .0

def derivatives(state, t):
	ds = np.zeros_like(state)

	_th = state[0]
	_Y = state[1]
	_th_h = state[2]
	_Y_h = state[3]

	ds[0] = state[1]
	ds[1] = - g / L * _th  - d * _Y + disturbance(t, 1.0)

	y = trim(_th, precision)

	ds[2] = _Y_h + k * (y - _th_h)
	ds[3] = - g / L * _th_h - d * _Y_h

	return ds

def solve_odeint(derivatives, state, t):
	solution = integrate.odeint(derivatives, state, t)	
	th = solution[:, 0]
	Y = solution[:, 1]
	th_h = solution[:, 2]
	Y_h = solution[:, 3]
	return (th, Y, th_h, Y_h)

def solve_manual(state, t, k):
	th = [state[0]]
	Y = [state[1]]
	th_h = [state[2]]
	Y_h = [state[3]]
	for i in range(len(t) - 1):
		th_1 = th[i] + Y[i] * dt
		Y_1 = Y[i] - (g / L * th[i] + d * Y[i]) * dt + disturbance(t[i], 1.0) * dt
		th_2 = th[i] + (Y_1 + Y[i]) * dt / 2
		Y_2 = Y[i] - (g / L * (th_1 + th[i]) + d * (Y_1 + Y[i])) * dt / 2 + disturbance(t[i], 1.0) * dt

		y = trim(th[i], precision)

		th_h_1 = th_h[i] + (Y_h[i] + k * (y - th_h[i])) * dt
		Y_h_1 = Y_h[i] - (g / L * th_h[i] + d * Y_h[i]) * dt
		th_h_2 = th_h[i] + (Y_h[i] + k * (y - th_h[i]) + Y_h_1 + k * (y - th_h_1)) * dt / 2
		Y_h_2 = Y_h[i] - (g / L * th_h[i] + d * Y_h[i] + g / L * th_h_1 + d * Y_h_1) * dt / 2

		th.append(th_2)
		Y.append(Y_2)
		th_h.append(th_h_2)
		Y_h.append(Y_h_2)
	return (th, Y, th_h, Y_h)

# th, Y, th_h, Y_h = solve_odeint(derivatives, state, t)
th, Y, th_h, Y_h = solve_manual(state, t, 10.0)

th_label, = pp.plot(t, th, label='Theta')
dth_label, = pp.plot(t, Y, label='dTheta')

oth_label, = pp.plot(t, th_h, label='Obsv(Theta)')
edth_label, = pp.plot(t, Y_h, label='Est(dTheta)')

pp.legend([th_label, dth_label, oth_label, edth_label])

pp.grid(True)
pp.show()