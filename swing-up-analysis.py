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
x = [.0]	# theta
y = [.1]	# angular velocity

a = [.0]	# cart position
b = [.0]	# cart speed

def e(x, y):
	return m * g * L * (1.0 - cos(x)) + m * (y * L) ** 2 / 2

def u(x, y):
	if e(x, y) >= 1.99 * m * g * L :
		return 0.0
	return 0.9 * y / m

def integrate(x, y):
	return - g / L * sin(x) + u(x, y) / L * cos(x)

for i in range(steps):
	x1 = x[i] + h * y[i]
	y1 = y[i] + h * integrate(x[i], y[i])

	x2 = x[i] + h * (y[i] + y1) * 0.5
	y2 = y[i] + h * (integrate(x[i], y[i]) + integrate(x1, y1)) * 0.5

	a1 = a[i] + h * b[i]
	b1 = b[i] + h * u(x[i], y[i])

	a2 = a[i] + h * (b[i] + b1) * 0.5
	b2 = b[i] + h * (u(x[i], y[i]) + u(x1, y1)) * 0.5

	t.append(t[i] + h)
	x.append(x2)
	y.append(y2)
	a.append(a2)
	b.append(b2)

def energy(x, y):
	e = []
	for i in range(len(x)):
		e.append(m * g * L * (1.0 - cos(x[i])) + m * (y[i] * L) ** 2 / 2)
	return e

pp.plot(t, x)
pp.plot(t, y)
pp.plot(t, a)
pp.plot(t, energy(x, y))
pp.grid(True)
pp.show()

