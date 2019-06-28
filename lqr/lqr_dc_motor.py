import numpy as np

from math import sin, cos, pi

from lqr_solver import lqr, dlqr

# physical constants
g = 9.8
l = 1.0
m_p = 0.5
m_k = 1.0
m_r = 0.001 # mass of the broom
k = 0.5 # motor torque constant
I = m_r * l * l / 12 # rotational inertia of a rod (m_r * l * l / 12)
R = 6.5 # motor resistance
r = 0.05   # shaft radius

L = (I + m_p * l * l) / (m_p * l)
M = m_k + m_p

a = 1.0 / (L * M - m_p * l)
b = k / (R * r)

A = np.matrix([
    [0.0, 1.0, 0.0, 0.0],
    [0.0, -a*L*R*b*b, -a*m_p*l*g, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, a*R*b*b, a*M*g, 0]
])

B = np.matrix([
    [0.0],
    [a*b],
    [0.0],
    [-a*b]
])

Q = np.matrix([
    [30.0,   .0,     .0,     .0],
    [.0,   1.,     .0,     .0],
    [.0,   .0,     100.,     .0],
    [.0,   .0,     .0,     1.0]    
])

R = np.matrix([1.0])

print("A = ", A)
print("B = ", B)

K, X, eig = lqr(A, B, Q, R)
print("K = ", list(K.A1))
print(X)
print(eig)