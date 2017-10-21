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

dt = 0.005
N = 2000

Kp = 25.0
Ki = 0.0
Kd = 2.0

Ei = 0.0

for i in range(0, N):

	e = th[i]
	de = dth[i]
	Ei = Ei + e * dt

	f.append((Kp * e + Ki * Ei + Kd * de))	

	d2th = (g * sin(th[i]) - f[i] * cos(th[i])) / L
	dth1 = dth[i] + d2th * dt
	
	th.append(th[i] + dth1 * dt)
	dth.append(dth1)

line_th, = plt.plot(range(0, N + 1), th, label = 'Th')
line_dth, = plt.plot(range(0, N + 1), dth, label = 'dTh')
line_f, = plt.plot(range(0, N + 1), f, label = 'acceleration')

plt.legend([line_th, line_dth, line_f])


plt.show()


