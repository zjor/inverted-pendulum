import matplotlib.pyplot as plt

from math import sin, cos, pi

'''
	Equations:
		Th'' = 1/L*(g*sin(Th) - x''*cos(Th))
		e(t) = Th(t)		
'''

L = 1.0
g = 9.8
th = [pi / 12]
dth = [0.0]
f = [0.0]
v = [0.0]
x = [0.0]

dt = 0.005
N = 2000

Kp = 45.0
Kd = 18.0

xKp = 2.0
xKd = 3.0

for i in range(0, N):

	f.append((Kp * th[i] + Kd * dth[i] + xKp * x[i] + xKd * v[i]))	

	d2th = (g * sin(th[i]) - f[i] * cos(th[i])) / L
	dth1 = dth[i] + d2th * dt

	v.append(v[i] + f[i] * dt)
	x.append(x[i] + v[i] * dt)
	
	th.append(th[i] + dth1 * dt)
	dth.append(dth1)

plt.figure(1)
plt.subplot(211)
line_th, = plt.plot(range(0, N + 1), th, label = 'Th')
line_dth, = plt.plot(range(0, N + 1), dth, label = 'dTh')
line_f, = plt.plot(range(0, N + 1), f, label = 'acceleration')

plt.legend([line_th, line_dth, line_f])

plt.subplot(212)
line_v, = plt.plot(range(0, N + 1), v, label = 'v')
line_x, = plt.plot(range(0, N + 1), x, label = 'x')

plt.legend([line_v, line_x])

plt.show()


