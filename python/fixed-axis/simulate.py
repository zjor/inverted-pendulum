import numpy as np
import matplotlib.pyplot as pp

from math import pi, sin, sqrt

g = 9.81
L = 1.0
m = 1.0
dt = 0.01
T_max = 10.0

times = np.arange(0, T_max, dt)
thetas = np.zeros(len(times))
omegas = np.zeros(len(times))

thetas[0] = 0.0   # initial Theta


def momentum(t):
  w = sqrt(g / L)

  return -0.5 * sin(w * t)


def derivatives(state, t):
  th, w = state[0], state[1]
  dth = w
  dw = - g / L * sin(th) - momentum(t) / (m * L**2)
  return [dth, dw]


def integrate(state, t, dt):
  """
  Integrate state with Heun's method
  """
  dstate = derivatives(state, t)
  _state = list(map(lambda x: x[0] + x[1] * dt, zip(state, dstate)))
  _dstate = derivatives(_state, t + dt / 2)
  return list(map(lambda x: x[0] + (x[1] + x[2]) * dt / 2, zip(state, dstate, _dstate)))


for i in range(1, len(times)):
  state = [thetas[i - 1], omegas[i - 1]]
  thetas[i], omegas[i] = integrate(state, times[i], dt)


pp.subplot(211)
pp.plot(times, omegas)
pp.plot(times, thetas)
pp.grid(True)

pp.subplot(212)
pp.plot(times, list(map(lambda t: momentum(t), times)))
pp.grid(True)

pp.show()







