import matplotlib.pyplot as plt

from math import sin, cos, pi

'''
	Equations:
		Th'' = 1/L*(g*sin(Th) - x''*cos(Th))
		e(t) = Th(t)		
'''

L = 1.0
g = 9.8
th = [0.2]
dth = [0.0]
f = [0.0]

dt = 0.1
N = 40

Kp = 10.0
Ki = 0.0
Kd = 20.0

Ei = 0.0

for i in range(0, N):

	e = th[i]
	de = dth[i]
	f.append(-(Kp * e + Ki * Ei + Kd * de))
	Ei = Ei + e * dt

	d2th = 1.0 / L * (g * sin(th[i]) - f[i] * cos(th[i]))
	dth1 = dth[i] + d2th * dt
	
	th.append(th[i] + dth1 * dt)
	dth.append(dth1)

line_th, = plt.plot(range(0, N + 1), th, label = 'Th')
line_dth, = plt.plot(range(0, N + 1), dth, label = 'dTh')
# line_f, = plt.plot(range(0, N + 1), f, label = 'f')

plt.legend([line_th, line_dth])


plt.show()


