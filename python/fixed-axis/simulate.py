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


def derivatives(state):
  th, w = state[0], state[1]
  dth = w
  dw = - g / L * sin(th)
  return [dth, dw]


def integrate(state, dt):
  """
  Integrate state with Heun's method
  """
  dstate = derivatives(state)
  _state = list(map(lambda x: x[0] + x[1] * dt, zip(state, dstate)))
  _dstate = derivatives(_state)
  return list(map(lambda x: x[0] + (x[1] + x[2]) * dt / 2, zip(state, dstate, _dstate)))


for i in range(1, len(times)):
  state = [thetas[i - 1], omegas[i - 1]]
  dstate = derivatives(state)
  thetas[i], omegas[i] = integrate(state, dt)



pp.plot(times, omegas)
pp.plot(times, thetas)
pp.grid(True)
pp.show()







