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

    Tl = r * m * a = r^2 * w' * m, where r - shaft radius, m - load mass
"""
import numpy as np
import matplotlib.pyplot as pp
import scipy.integrate as integrate
from math import pi, sin, cos

# motor constants
L = 0.1
R = 6.5
k = 0.5
J = 0.2
B = 0.3
U = 12.0
m = 10.0
r = 0.0025

# Coulomb's friction
Fc = 0.4

dt = 0.01
t = np.arange(0, 10.0, dt)
w = np.zeros(len(t))
i = np.zeros(len(t))

# L/R << J/B, => i = U/R - k/R * w
w2 = np.zeros(len(t))

for j in range(0, len(t) - 1):
    U = 12.0 * sin(t[j] * pi / 4)
    # if t[j] > 5.0:
    #     U = .0

    di = (U - R * i[j] - k * w2[j]) / L
    
    dw2 = (U * k / R - w2[j] * (B + k * k / R) - Fc * np.sign(w2[j])) / (J + m * r * r)

    i[j + 1] = i[j] + di * dt
    w2[j + 1] = w2[j] + dw2 * dt

pp.plot(t, i, 'r')
pp.plot(t, w2, 'b')
pp.grid(True)
pp.show()





