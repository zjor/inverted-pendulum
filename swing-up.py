import matplotlib.pyplot as plt

from math import sin, cos, pi, sqrt

'''
	Equations:
		Th'' = 1/L*(g*sin(Th) - x''*cos(Th))
		e(t) = Th(t)		
'''

L = 1.0
g = 9.8
th = [pi]
dth = [0.0]
f = [0.0]
v = [0.0]
x = [0.0]
x0 = 0.0
max_th = abs(pi - th[0])

dt = 0.01
N = 4000

def agm(x, y):
	a, g = x, y
	while abs(a - g) > 0.001:
		a = (a + g) / 2.0
		g = sqrt(a * g)	
	return a

def getResonanceFreq(th):
	return agm(1.0, cos(th / 2.0)) * sqrt(g / L)

def getControl(theta, omega, x, v, t):
	return cos(getResonanceFreq(max_th) * t)

for i in range(0, N):

	f.append(getControl(th[i], dth[i], x[i], v[i], i * dt))

	d2th = (g * sin(th[i]) - f[i] * cos(th[i])) / L
	dth1 = dth[i] + d2th * dt

	v.append(v[i] + f[i] * dt)
	x.append(x[i] + v[i] * dt)
	
	new_th = th[i] + dth1 * dt
	th.append(new_th)
	dth.append(dth1)
	max_th = max(max_th, abs(pi - new_th))

print("Max theta: ", max_th)

plt.figure(1)
plt.subplot(211)
plt.title("PD regulator")
line_th, = plt.plot(range(0, N + 1), th, label = 'Th')
line_dth, = plt.plot(range(0, N + 1), dth, label = 'dTh')
line_f, = plt.plot(range(0, N + 1), f, label = 'acceleration')

plt.legend([line_th, line_dth, line_f])

plt.subplot(212)
line_v, = plt.plot(range(0, N + 1), v, label = 'v')
line_x, = plt.plot(range(0, N + 1), x, label = 'x')

plt.legend([line_v, line_x])
plt.show()