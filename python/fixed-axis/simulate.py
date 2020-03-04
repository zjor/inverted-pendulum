import numpy as np
import matplotlib.pyplot as pp

from math import pi, sin

g = 9.81
L = 1.0
dt = 0.01
T_max = 10.0

times = np.arange(0, T_max, dt)
thetas = np.zeros(len(times))
omegas = np.zeros(len(times))

thetas[0] = pi/12   # initial Theta

for i in range(1, len(times)):
  dth = omegas[i - 1]
  dw = -g / L * sin(thetas[i - 1])
  omegas[i] = omegas[i - 1] + dw * dt
  thetas[i] = thetas[i - 1] + dth * dt

pp.plot(times, omegas)
pp.plot(times, thetas)
pp.grid(True)
pp.show()







