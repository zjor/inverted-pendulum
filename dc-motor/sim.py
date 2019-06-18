"""
    DC-motor simulation

    Equations of the motor:
        U = L * i' + R * i + k * w
        J * w' + B * w = k * i - Tl
    where:
        U - input voltage
        L - rotor inductance
        R - rotor resistance
        k * w - back EMF, w - rotor angular velocity
        J - rotor momentum of intertia
        B - rotor friction
        Tl - external load torque

    Tl = r * m * a = r^2 * w * m, where r - shaft radius, m - load mass
"""
import numpy as np
import matplotlib.pyplot as pp
import scipy.integrate as integrate

# motor constants
L = 0.1
R = 6.5
k = 0.5
J = 0.2
B = 0.3
dt = 0.01
U = 12.0
m = 10.0
r = 0.0025

t = np.arange(0, 10.0, dt)
w = np.zeros(len(t))
i = np.zeros(len(t))

# L/R << J/B, => i = U/R - k/R * w
w2 = np.zeros(len(t))

for j in range(0, len(t) - 1):
    if t[j] > 5.0:
        U = .0

    Tl = m * w[j] * r * r
    di = (U - R * i[j] - k * w[j]) / L
    dw = (k * i[j] - B * w[j] - Tl) / J

    Tl = m * w2[j] * r * r
    dw2 = U * k / (J * R) - Tl / J - w2[j] * (B / J + k * k / (J * R))

    i[j + 1] = i[j] + di * dt
    w[j + 1] = w[j] + dw * dt
    w2[j + 1] = w2[j] + dw2 * dt

pp.plot(t, i, 'r')
pp.plot(t, w, 'b')
pp.plot(t, w2, 'k')
pp.grid(True)
pp.show()





