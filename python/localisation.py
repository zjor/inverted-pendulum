import matplotlib.pyplot as plt

'''
U[x''] -> x
x'(0) = v0
x''(0) = a0
x(0) = x0

The goal is to return to x1
'''

x0 = 10.0
v0 = 0.0
a0 = 0.0
dt = 0.1

x = [x0]
v = [v0]
a = [a0]

Kp = 2.0
Ki = 0.5
Kd = 2.0

Ei = 0.0

N = 200

for i in range(0, N):
	x1 = x[i] + v[i] * dt
	v1 = v[i] + a[i] * dt
	a1 = - (Kp * x1 + Ki * Ei + Kd * v1)
	Ei = Ei + x1 * dt

	x.append(x1)
	v.append(v1)
	a.append(a1)

line_x, = plt.plot(range(0, N + 1), x, label = 'x')
line_v, = plt.plot(range(0, N + 1), v, label = 'v')
line_a, = plt.plot(range(0, N + 1), a, label = 'a')

plt.legend([line_x, line_v, line_a])

# plt.ylim(-10, 20)

plt.show()





