import matplotlib.pyplot as plt

from math import sin, pi

# x'' = -x

h = 0.01
n = 10000

x = [1.0]
v = [0.0]

for i in range(n):

	v1 = v[i] - h * x[i]
	x1 = x[i] + h * v[i]

	v.append(v[i] + h * (-x[i] - x1) / 2.0)
	x.append(x[i] + h * (v[i] + v1) / 2.0)



plt.plot(x, v)
plt.show()

