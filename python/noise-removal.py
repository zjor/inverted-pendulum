import matplotlib.pyplot as pp
from math import sin, pi
import numpy as np

def f(x):
	return sin(x) + 0.2 * sin(8.0 * x)

# https://terpconnect.umd.edu/~toh/spectrum/Smoothing.html
def smooth9(values):
	k = [0.02376257744, 0.06195534498, 0.1228439993, 0.185233293, 0.2124095706, 0.185233293, 0.1228439993, 0.06195534498, 0.02376257744]
	smoothed = np.zeros(len(values))
	for i in range(0, len(values)):
		v = .0
		for j in range(0, len(k)):
			if i > j:
				v += values[i - j] * k[j]
		smoothed[i - len(k)] = v
	return smoothed

# https://en.wikipedia.org/wiki/Low-pass_filter
def lowPass(values, cutoff):
	out = np.zeros(len(values))
	out[0] = values[0]
	for i in range(1, len(values)):
		out[i] = out[i - 1] + cutoff * (values[i] - out[i - 1])
	return out

def firstOrderLowPass(x, p):
	y = np.zeros(len(x))
	y[0] = x[0]
	for i in range(1, len(x)):
		y[i] = 0.5 * (1.0 - p) * (x[i] + x[i - 1]) - p * y[i - 1]
	return y

x = np.arange(.0, 4.0 * pi, 0.2)
y = np.vectorize(f)(x)
y9 = smooth9(y)
yL = lowPass(y, 0.5)
yL2 = firstOrderLowPass(y, 0.1)

pp.plot(x, y9, label='Smooth9')
pp.plot(x, y, label='Original')
pp.plot(x, yL, label='Low-Pass')
pp.plot(x, yL2, label='First Order Low-Pass')
pp.legend()
pp.show()



