import numpy as np
import matplotlib.pyplot as plt

from math import sin, cos, pi

from lqr_solver import lqr, dlqr

"""
	Equations:
		Th'' = 1/L*(g*sin(Th) - x''*cos(Th))
		e(t) = Th(t)		
"""

g = 9.81
L = 0.6

A = np.matrix([
    [.0,    1.,     .0,     .0],
    [g / L, .0,     .0,     .0],
    [.0,    .0,     .0,     1.],
    [.0,    .0,     .0,     .0]
])

B = np.matrix([
[.0],
[-1. / L],
[.0],
[1.]
])

Q = np.matrix([
    [10000.,   .0,     .0,     .0],
    [.0,   5.,     .0,     .0],
    [.0,   .0,     1.,     .0],
    [.0,   .0,     .0,     1.]    
])

R = np.matrix([20.])

K, X, eig = lqr(A, B, Q, R)
print(K)
print(X)
print(eig)

th = [pi / 12]
dth = [0.0]
f = [0.0]
v = [0.0]
x = [0.0]
x0 = 0.0

dt = 0.005
N = 10000


for i in range(0, N):

	X = np.matrix([[th[i], dth[i], x[i] - x0, v[i]]]).T

	f.append((- K * X)[0, 0])	

	d2th = (g * sin(th[i]) - f[i] * cos(th[i])) / L
	dth1 = dth[i] + d2th * dt

	v.append(v[i] + f[i] * dt)
	x.append(x[i] + v[i] * dt)
	
	th.append(th[i] + dth1 * dt)
	dth.append(dth1)

	if i == 5000:
		x0 = 5.0

plt.figure(1)
plt.subplot(211)
plt.title("LQR regulator")
line_th, = plt.plot(range(0, N + 1), th, label = 'Th')
line_dth, = plt.plot(range(0, N + 1), dth, label = 'dTh')
line_f, = plt.plot(range(0, N + 1), f, label = 'acceleration')

plt.legend([line_th, line_dth, line_f])

plt.subplot(212)
line_v, = plt.plot(range(0, N + 1), v, label = 'v')
line_x, = plt.plot(range(0, N + 1), x, label = 'x')

plt.legend([line_v, line_x])
plt.show()