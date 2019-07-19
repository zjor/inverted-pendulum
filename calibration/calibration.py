"""
Calibration algorithm for incremental rotary encoder.
The goal is to find zero position of fading oscillations.

Parameters:
	l - length of the rod
	m - mass of the pendulum
	b - friction coefficient
    th - pendulum angle

Equation of motion:
    th'' + (b / m) * th' + (g / l) * sin(th) = 0

System:
	th' = Y,
	Y' = (b / m) * Y + (g / l) * sin(th)

State:
	[th, Y]

Conclusion:
    Option A: Wait until there is no motion for 2-3 seconds, take current position as zero.
    Option B: Remember positions when the speed changes sign 
    (those are maximums and minimums of angle) and average them using sliding window.

"""
import matplotlib
matplotlib.use('TKAgg')

import numpy as np
import matplotlib.pyplot as pp
import scipy.integrate as integrate

from math import pi
from numpy import sin, cos

# physical constants
g = 9.81
l = 1.0
m = 0.5
b = 0.1

# simulation time
dt = 0.05
Tmax = 30
t = np.arange(0.0, Tmax, dt)

# initial conditions
Y = .0 		# pendulum angular velocity
th = pi/3	# pendulum angle

state = np.array([th, Y])

def f(t):
    if t > 1.0 and t < 1.5:
        return 10.0
    else:
        return 0.0

def derivatives(state, t):
    ds = np.zeros_like(state)

    _th, _Y = state

    ds[0] = _Y
    ds[1] = - (b / m) * _Y - (g / l) * sin(_th) + f(t)

    return ds

solution = integrate.odeint(derivatives, state, t)
ths = solution[:, 0] + 1.5
ws = solution[:, 1]

def get_zero(ths, ws):
    mean = 0.0
    n = 0
    for i in range(0, len(ws) - 1):
        if ws[i] * ws[i + 1] <= 0:
            mean = mean + ths[i]
            n = n + 1
    return mean / n

n = len(ths) / 5
print("Mean: %f" % (sum(ths[:-n]) / n))
print("Zero: %f" % get_zero(ths, ws))

pp.plot(t, ths)
pp.plot(t, ws)
pp.grid(True)
pp.show()

