'''
Simulation of forced oscillations
Equation:
	Th'' = - g/L * sin(Th) + F / (m * L) * cos(Th)
'''

import matplotlib.pyplot as pp
import numpy as np

from math import sin, cos, pi

g = 9.81
L = 1.0
m = 1.0
h = 0.01
steps = 2000

t = [.0]
x = [.1]
y = [.0]

def force(x, y):
	return 0.5 * y

def integrate(x, y):
	return - g / L * sin(x) + force(x, y) / (m * L) * cos(x)

for i in range(steps):
	x1 = x[i] + h * y[i]
	y1 = y[i] + h * integrate(x[i], y[i])

	x2 = x[i] + h * (y[i] + y1) * 0.5
	y2 = y[i] + h * (integrate(x[i], y[i]) + integrate(x1, y1)) * 0.5

	t.append(t[i] + h)
	x.append(x2)
	y.append(y2)

def energy(x, y):
	e = []
	for i in range(len(x)):
		e.append(m * g * L * (1.0 - cos(x[i])) + m * (y[i] * L) ** 2 / 2)
	return e

pp.plot(t, x)
pp.plot(t, y)
pp.plot(t, energy(x, y))
pp.grid(True)
pp.show()

