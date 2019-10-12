import numpy as np
import matplotlib.pyplot as pp

dt = 5.0/1000
times = np.arange(0, 5.0, dt)
xs = np.zeros(len(times))
vs = np.zeros(len(times))
us = np.zeros(len(times))
xs[0] = 0.1

f_a = 35.98
f_b = 2.22
f_c = -2.79

for i in range(len(times) - 1):
    x, v = xs[i], vs[i]
    a = - (1.0 * x + 0.1 * v)
    u = (a + f_a * v - f_c * np.sign(v)) / f_b

    dx = v
    dv = a # -f_a * v + f_b * u + f_c * np.sign(v)

    xs[i + 1] = x + dx * dt
    vs[i + 1] = v + dv * dt
    us[i + 1] = u

pp.plot(times, xs, label="x")
pp.plot(times, vs, label="v")
pp.plot(times, us, label="u")
pp.legend()
pp.grid(True)
pp.show()
