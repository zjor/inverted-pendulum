import numpy as np
import matplotlib.pyplot as pp
import matplotlib.animation as animation

from math import pi, sqrt
from numpy import sin, cos


g = 9.81
L = 1.0
m = 1.0
dt = 0.01
T_max = 10.0

times = np.arange(0, T_max, dt)
thetas = np.zeros(len(times))
omegas = np.zeros(len(times))

thetas[0] = 0   # initial Theta


def momentum(t):
  w = sqrt(g / L)

  return -0.5 * sin(w * t)


def derivatives(state, t):
  """
  Th'' = - g / L * sin(Th) - M(t) / (m * L^2)
  """
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

# Animation

xs = L * sin(thetas)
ys = - L * cos(thetas)

fig = pp.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1.3, 1.3), ylim=(-1.2, 1.2))
ax.set_aspect('equal')
ax.grid()

line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)


def init():
    line.set_data([], [])
    time_text.set_text('')
    return line, time_text


def animate(i):
    line.set_data([0, xs[i]], [0, ys[i]])
    time_text.set_text(time_template % (i*dt))    
    return line, time_text

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(times)),
                              interval=25, blit=True, init_func=init)

pp.show()







