import numpy as np
import matplotlib.pyplot as pp

from math import sin, cos, pi

from pidcontroller import PIDController

'''
	Equations:
		Th'' = 1/L*(g*sin(Th) - x''*cos(Th))
		e(t) = Th(t)		
'''

L = 1.0
g = 9.8
th = [pi / 24]
w = [0.0]
f = [0.0]
v = [0.0]
x = [0.0]
x0 = 0.0

dt = 0.01
time = np.arange(0, 10, dt)

Kp = 60.0
Kd = 14.0
Ki = 0.0

xKp = 2.0
xKd = 3.0
xKi = 0.0

pid_th = PIDController(Kp, Kd, Ki, 0.0, th[0])
pid_x = PIDController(xKp, xKd, xKi, 0.0, x[0])

for i in range(0, len(time) - 1):

	u = pid_th.getControl(th[i], dt) + pid_x.getControl(x[i], dt)
	f.append(-u)

	d2th = (g * sin(th[i]) - f[i] * cos(th[i])) / L
	dth1 = w[i] + d2th * dt

	v.append(v[i] + f[i] * dt)
	x.append(x[i] + v[i] * dt)
	
	th.append(th[i] + dth1 * dt)
	w.append(dth1)

	if i == len(time) / 2:
		x0 = 0.2
		pid_x.setTarget(x0)

pp.figure(1)
pp.subplot(211)
pp.title("PD regulator")
line_th, = pp.plot(time, th, label = 'th')
line_dth, = pp.plot(time, w, label = 'w')
pp.grid(True)
pp.legend([line_th, line_dth])

pp.subplot(212)
line_v, = pp.plot(time, v, label = 'v')
line_x, = pp.plot(time, x, label = 'x')
line_f, = pp.plot(time, f, label = 'u')
pp.grid(True)
pp.legend([line_v, line_x, line_f])
pp.show()


