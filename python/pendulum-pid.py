import numpy as np
import matplotlib.pyplot as plt

from math import sin, cos, pi
from pidcontroller import PIDController

'''
	Equations:
		Th'' = 1/L*(g*sin(Th) - x''*cos(Th))
		e(t) = Th(t)		
'''

L = 1.0
g = 9.8
ths = [-pi / 12]
ws = [0.0]
us = [0.0]
vs = [0.0]
xs = [0.0]
x0 = 0.0

dt = 0.01
time = np.arange(0, 10, dt)

Kp = 10.0
Kd = 5.0
Ki = 0.0

xKp = 0.0
xKd = 0.0
xKi = 0.0

pid_th = PIDController(Kp, Kd, Ki, 0.0)
pid_x = PIDController(xKp, xKd, xKi, 0.0)

for i in range(0, len(time) - 1):

	th = ths[-1]
	w = ws[-1]
	x = xs[-1]
	v = vs[-1]

	# f.append((Kp * th[i] + Kd * dth[i] + xKp * (x[i] - x0) + xKd * v[i]))
	# u = -pid_th.getControl(th, dt) + pid_x.getControl(x, dt)
	u = -(Kp * th + Kd * w)
	print(u)

	us.append(u)	

	d2th = (g * sin(th) - u * cos(th)) / L
	dth1 = w + d2th * dt

	vs.append(v + u * dt)
	xs.append(x + v * dt)
	
	ths.append(th + dth1 * dt)
	ws.append(dth1)

	# if i == len(time) / 2:
	# 	x0 = 0.2

plt.figure(1)
plt.subplot(211)
plt.title("PD regulator")
line_th, = plt.plot(time, ths, label = 'th')
line_dth, = plt.plot(time, ws, label = 'w')

plt.legend([line_th, line_dth])
plt.grid(True)

plt.subplot(212)
line_v, = plt.plot(time, vs, label = 'v')
line_x, = plt.plot(time, xs, label = 'x')
line_u, = plt.plot(time, us, label = 'u')

plt.legend([line_v, line_x, line_u])
plt.grid(True)
plt.show()


