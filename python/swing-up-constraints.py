"""
Simulation of pendulum swing-up by energy control with constraint by x

Equations:
	th'' = (g * sin(th) - u * cos(th)) / L,
	u = k * E * th' * cos(th),
	where E = m * (th' * L) ^ 2 / 2 + m * g * L * (cos(th) - 1), zero-energy is in upright position

System:
	th' = Y,
	Y' = (g * sin(th) - u * cos(th)) / L,
	x' = Z,
	Z' = u = k * E * Y * cos(th),

State: 
	[th, Y, x, Z]

References:
- Swinging up a pendulum by energy control - K.J. Astrom, K. Furuta
"""

import numpy as np

import matplotlib
matplotlib.use('TKAgg')

import matplotlib.pyplot as pp
import scipy.integrate as integrate
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

from math import pi
from numpy import sin, cos, sign

# physical constants
g = 9.8
L = 1.0
m = 0.5

# simulation time
dt = 0.051
Tmax = 50
t = np.arange(0.0, Tmax, dt)

# initial conditions
Y = .0 		# pendulum angular velocity
th = pi		# pendulum angle
x = .0		# cart position
x0 = 0		# desired cart position
Z = 0.00	# cart velocity
k = 0.055	# control gain coefficient
E0 = -2.0 * m * g * L # starting energy

state = np.array([th, Y, x, Z])

def energy(th, dth):
	return m * dth * L * dth * L / 2 + m * g * L * (cos(th) - 1)

def get_control(x, v, th, w, e):
	Kx = 2.0
	Kv = 1.0
	# u = -(Kx * (_x - 0.1) + Kv * _Z)
	# u = - 0.5 * sign(_Y * cos(_th))

	if th >= 0.5 * pi and th <= 1.5 * pi:
		# u = k * E * _Y * cos(_th)
		if x >= 0.1:
			u = -(Kx * (x + 0.1) + Kv * v)
		elif x <= -0.1:
			u = -(Kx * (x - 0.1) + Kv * v)
		else:
			u = - 0.5 * sign(w * cos(th))
	else:
		u = -(Kx * x + Kv * v)
	return u

def get_position_control(x, v, x0):
	Kx = 4.0
	Kv = 1.0
	return -(Kx * (x - x0) + Kv * v)

pid_i = 0.0
Ki = 1.0
Kp = 25.0
def get_velocity_control(v, target, dt):
	global pid_i

	error = target - v
	# pid_i += Ki * error * dt
	return Kp * error + pid_i


GO_CENTER = 0
SWING_UP = 1
SOFT_STOP = 2
DONE = 3


TOLERANCE = 1e-2

fsm_state = GO_CENTER
def get_state_control(x, v, th, w, e):
	global fsm_state
	u = 0.0

	if fsm_state == GO_CENTER:

		if e >= 0.0:
			fsm_state = DONE 
		elif abs(x) <= TOLERANCE:
			fsm_state = SWING_UP
		else:
			u = get_position_control(x, v, 0.0)

	elif fsm_state == SWING_UP:
		if e >= 0.0:
			fsm_state = DONE
		elif abs(x) <= 0.05:
			u = -0.5 * sign(w * cos(th))
		else:
			fsm_state = GO_CENTER
	elif fsm_state == DONE:
		u = get_position_control(x, v, 0.0)
		
	return u

fsm_state = SWING_UP
def get_state_control2(x, v, th, w, e, dt):
	global fsm_state
	u = 0.0
	going_to_center = sign(w * cos(th)) * x > 0

	if fsm_state == SWING_UP:
		if e >= 0:
			fsm_state = DONE
		elif (abs(x) <= 0.1 or going_to_center) and (th >= 2 * pi / 8 and th <= 10.0 * pi / 8):
			u = -0.5 * sign(w * cos(th))
		else:
			fsm_state = SOFT_STOP			
	elif fsm_state == SOFT_STOP:
		if going_to_center:
			fsm_state = SWING_UP
		else:
			u = get_velocity_control(v, 0.0, dt)
	elif fsm_state == DONE:
		u = get_velocity_control(v, 0.0, dt)

	return u


_last_integration_time = 0
def derivatives(state, t):
	global _last_integration_time
	dt = t - _last_integration_time
	_last_integration_time = t

	ds = np.zeros_like(state)

	_th = state[0]
	_Y = state[1]	# th'
	_x = state[2]
	_Z = state[3]	# x'

	E = energy(_th, _Y)

	# u = get_control(_x, _Z, _th, _Y, E)
	# u = get_state_control(_x, _Z, _th, _Y, E)
	u = get_state_control2(_x, _Z, _th, _Y, E, dt)

	ds[0] = state[1]
	ds[1] = (g * sin(_th) - u * cos(_th)) / L
	ds[2] = state[3]
	ds[3] = u

	return ds

print("Integrating...")
# integrate your ODE using scipy.integrate.
solution = integrate.odeint(derivatives, state, t)
print("Done")

ths = solution[:, 0]
Ys = solution[:, 1]
xs = solution[:, 2]
vs = solution[:, 3]

pxs = L * sin(ths) + xs
pys = L * cos(ths)

fig = pp.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1.5, 1.5), ylim=(-1.2, 1.2))
ax.set_aspect('equal')
ax.grid()

patch = ax.add_patch(Rectangle((0, 0), 0, 0, linewidth=1, edgecolor='k', facecolor='g'))

line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

energy_template = 'E = %.3f J'
energy_text = ax.text(0.05, 0.8, '', transform=ax.transAxes)

cart_width = 0.3
cart_height = 0.2

def init():
    line.set_data([], [])
    time_text.set_text('')
    energy_text.set_text('')

    patch.set_xy((-cart_width/2, -cart_height/2))
    patch.set_width(cart_width)
    patch.set_height(cart_height)
    return line, time_text, energy_text, patch


def animate(i):
    thisx = [xs[i], pxs[i]]
    thisy = [0, pys[i]]

    line.set_data(thisx, thisy)
    time_text.set_text(time_template % (i*dt))
    
    E = energy(ths[i], Ys[i])
    energy_text.set_text(energy_template % (E))

    patch.set_x(xs[i] - cart_width/2)
    return line, time_text, energy_text, patch

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(solution)),
                              interval=25, blit=True, init_func=init)


pp.figure()
pp.subplot(211)

Es = np.vectorize(energy)(ths, Ys)
Us = k * Es * Ys * cos(ths)

pp.plot(t, Us, label='U')
pp.plot(t, vs, label='v')
pp.plot(t, xs, label='x')
# pp.plot(t, ths, label="th")
# pp.plot(t, Ys, label="th'")
pp.grid(True)
pp.legend()
pp.subplot(212)
pp.plot(t, Es, label='E')
pp.grid(True)
pp.legend()
pp.show()


# Set up formatting for the movie files
# Writer = animation.writers['ffmpeg']
# writer = Writer(fps=15, metadata=dict(artist='Sergey Royz'), bitrate=1800)
# ani.save('controlled-cart.mp4', writer=writer)