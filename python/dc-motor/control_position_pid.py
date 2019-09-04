"""
    Controlling cart position with DC motor and PID
    Assumptions:
        - no friction
        - L / R << J / B
    Cart state: [x, x']
    System state: [w, x]
    Evolution:
    x' = w * r
    w' = (U * k / R - w * (B + k^2 / R)) / (J + m * r^2)
    Control: U(x, w)
"""

import numpy as np
import matplotlib.pyplot as pp

from control_position_lqr import limit_control

# motor constants
L = 0.1
R = 6.5
k = 0.5
J = 0.2
B = 0.3
U = 12.0
m = 10.0
r = 0.005

Kp = 100.0
Kd = 10.0
Ki = 10.0

a = -(B + k * k / R) / (J + m * r * r)
b = k / ((J + m * r * r) * R)

dt = 0.005
t = np.arange(0.0, 30.0, dt)
x = np.zeros(len(t))
w = np.zeros(len(t))
control = np.zeros(len(t))

x[0] = 0.1
last_error = None
error_integral = 0.0

if __name__ == "__main__":
    for i in range(0, len(t) - 1):
        error = x[i]
        if last_error == None:
            last_error = x[i]
        u = - limit_control(Kp * error + Kd * (error - last_error) / dt + Ki * error_integral, 12)
        last_error = error
        error_integral += error * dt
        
        control[i] = u

        dx = w[i] * r
        dw = a * w[i] + b * u

        x[i + 1] = x[i] + dx * dt
        w[i + 1] = w[i] + dw * dt

    plot_x = pp.subplot(311)
    plot_x.title.set_text("Position")
    plot_x.grid(True)
    x_line, = pp.plot(t, x, 'r', label="x")

    plot_w = pp.subplot(312)
    plot_w.title.set_text("Angular velocity")
    plot_w.grid(True)
    w_line, = pp.plot(t, w, 'b', label="w")

    plot_u = pp.subplot(313)
    plot_u.title.set_text("Control")
    plot_u.grid(True)
    control_line, = pp.plot(t, control, 'g', label="u")    

    pp.show()
