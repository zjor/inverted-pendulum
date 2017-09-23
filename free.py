import matplotlib.pyplot as plt

from math import sin, pi

'''
	x'' = -x

	x' = y; f(t, x, y) = y
	y' = -x; g(t, x, y) = -x

'''

h = 0.1
n = 10000

x = [1.0]
y = [0.0]

for i in range(n):

	x1 = x[i] + h * y[i]
	y1 = y[i] - h * x[i]

	new_x = x[i] + h * (y[i] + y1) / 2
	new_y = y[i] - h * (x[i] + x1) / 2

	# k1 = y[i]
	# k2 = y[i] + k1 / 2
	# k3 = y[i] + k2 / 2
	# k4 = y[i] + k3
	# new_x = x[i] + h * h * (k1 + 2 * k2 + 2 * k3 + k4) / 6

	# k1 = -x[i]
	# k2 = -x[i] - k1 / 2
	# k3 = -x[i] - k2 / 2
	# k4 = -x[i] - k3
	# new_y = y[i] + h * h * (k1 + 2 * k2 + 2 * k3 + k4) / 6


	y.append(new_y)
	x.append(new_x)



plt.plot(x, y)
plt.show()

