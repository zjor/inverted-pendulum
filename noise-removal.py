import matplotlib.pyplot as pp
from math import sin, pi
import numpy as np

def f(x):
	return sin(x) + 0.2 * sin(8.0 * x)

# https://terpconnect.umd.edu/~toh/spectrum/Smoothing.html
def smooth9(values):
	k = [0.02376257744, 0.06195534498, 0.1228439993, 0.185233293, 0.2124095706, 0.185233293, 0.1228439993, 0.06195534498, 0.02376257744]
	smoothed = np.zeros(len(values) - len(k))
	for i in range(len(k), len(values)):
		v = .0
		for j in range(0, len(k)):
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

x = np.arange(.0, 4.0 * pi, 0.2)
y = np.vectorize(f)(x)
y9 = smooth9(y)
yL = lowPass(y, 0.5)

pp.plot(x[9:], y9, label='Smooth9')
pp.plot(x, y, label='Original')
pp.plot(x, yL, label='Low-Pass')
pp.legend()
pp.show()



