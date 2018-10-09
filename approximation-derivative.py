"""
	This code shows how to calculate derivative by 3 points.
	It approximatex signal by 3 points as a parabolic function then
	take derivative of that polynom
"""
import numpy as np

from math import pi, sin, cos

from matplotlib import pyplot


def solve_p2(x, y):
	det = (x[0] * x[0] - x[0] * (x[1] + x[2]) + x[1] * x[2]) * (x[1] - x[2])
	detA = y[0] * (x[1] - x[2]) - x[0] * (y[1] - y[2]) + (y[1] * x[2] - x[1] * y[2])
	detB = x[0] * x[0] * (y[1] - y[2]) - y[0] * (x[1] * x[1] - x[2] * x[2]) + (x[1] * x[1] * y[2] - x[2] * x[2] * y[1])
	detC = x[0] * x[0] * (x[1] * y[2] - x[2] * y[1]) - x[0] * (x[1] * x[1] * y[2] - x[2] * x[2] * y[1]) + y[0] * (x[1] * x[1] * x[2] - x[1] * x[2] * x[2])
	if det != 0:
		return (detA / det, detB / det, detC / det)
	else:
		raise Error("det == 0")

def poly(a, b, c, x):
	return a * x * x + b * x + c

def derivative(a, b, x):
	return 2.0 * a * x + b

def f(x):
	return int((sin(x) + 1.0) / 2.0 * 1024)

def df(x):
	return int((cos(x)) / 2.0 * 1024)


def old_derivative(ts, xs):
	ds = np.zeros(ts.size)
	for i in range(1, ts.size):
		ds[i] = (xs[i] - xs[i - 1]) / (ts[i] - ts[i - 1])
	ds[0] = ds[1]
	return ds

def new_derivative(ts, xs):
	ds = np.zeros(ts.size)
	for i in range(2, ts.size):
		x = [ts[i - 2], ts[i - 1], ts[i]]
		y = [xs[i - 2], xs[i - 1], xs[i]]
		a, b, c = solve_p2(x, y)
		ds[i] = derivative(a, b, ts[i])
	ds[0] = ds[1] = ds[2]		
	return ds

ts = np.arange(0.0, 10.0 * pi, 0.5)
xs = np.vectorize(f)(ts)
ys = np.vectorize(df)(ts)
ds = old_derivative(ts, xs)
ds1 = new_derivative(ts, xs)

pyplot.plot(ts, xs, 'ob')
pyplot.plot(ts, ds, 'or')
pyplot.plot(ts, ds1, 'og')
pyplot.plot(ts, ys, 'oy')
pyplot.show()