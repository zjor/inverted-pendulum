import matplotlib.pyplot as plt

from math import sin, cos, pi, sqrt

'''
	x'' = -x

	x' = y; f(t, x, y) = y
	y' = -x; g(t, x, y) = -x

'''

h = 0.001
n = 30000

t = [0]
x = [0.0]
y = [0.0]

def force(t, x, y):
	return 0.5 * sin(0.5*sqrt(t**3))

for i in range(n):
	
	x1 = x[i] + h * y[i]
	y1 = y[i] + h * force(t[-1], x[i], y[i])

	x2 = x[i] + h * (y[i] + y1) / 2
	y2 = y[i] + h * (force(t[-1], x[i], y[i]) + force(t[-1], x1, y1)) / 2

	y.append(y2)
	x.append(x2)
	t.append(t[-1] + h)

# plt.figure(1)

# plt.subplot(211)
line_x, = plt.plot(t, x, label='x')
# plt.subplot(212)
line_y, = plt.plot(t, y, label='y')

plt.legend([line_x, line_y])
plt.grid()
plt.show()

