"""
    Controlling cart position with DC motor and LQR
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

from lqr_solver import lqr, dlqr

# motor constants
L = 0.1
R = 6.5
k = 0.5
J = 0.2
B = 0.3
U = 12.0
m = 10.0
r = 0.005

a = -(B + k * k / R) / (J + m * r * r)
b = k / ((J + m * r * r) * R)

A = np.matrix([    
    [a, .0],
    [r, .0]
])

B = np.matrix([
    [b],
    [0.0]
    ])

Q = np.matrix([
    [0.01, 0.0],
    [0.0, 1000.0]
])

R = np.matrix([0.01])

K, X, eig = lqr(A, B, Q, R)

print("K:")
print(K)
print("X:")
print(X)
print("Eigen vectors:")
print(eig)

dt = 0.005
t = np.arange(0.0, 30.0, dt)
x = np.zeros(len(t))
w = np.zeros(len(t))

x[0] = 0.1

for i in range(0, len(t) - 1):
    u = - (K[0,0] * w[i] + K[0,1] * x[i])
    dx = w[i] * r
    dw = a * w[i] + b * u

    x[i + 1] = x[i] + dx * dt
    w[i + 1] = w[i] + dw * dt

pp.plot(t, x, 'r')
pp.plot(t, w, 'b')
pp.grid(True)
pp.show()
