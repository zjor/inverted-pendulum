import math
import matplotlib.pyplot as plt

from math import pi

r = 0.0135
k = pi * r / 100
a = 5.0

v = [0.0]
d = []
dt = 1e-2

t = range(0, 100)
for i in t:
	v_new = v[i] + a * dt
	d_new = k / v_new * 1e3

	v.append(v_new)
	d.append(d_new)



line_x, = plt.plot(t, d, label='d')
line_y, = plt.plot(t, v[1:], label='v')

plt.legend([line_x, line_y])
plt.show()

