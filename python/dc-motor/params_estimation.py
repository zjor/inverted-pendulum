"""
Motor state is described by the system of equations:
x' = r * w,
w' = -a * w + b * u

where a, b are motor params and can be determined based on experimental data

w(t) = b * u / a * (1 - exp(-a * t))
"""
import numpy as np
import matplotlib.pyplot as pp

from math import exp

def w_t(t, a, b, u):
    return b * u / a * (1 - exp(-a * t))

def generate_sample(times, u, a, b, with_noise=True):
    if with_noise:
        noise = np.random.normal(0.0, 0.5, len(times))
    else:
        noise = np.zeros(len(times))

    ws = []
    for t in times:
        ws.append(w_t(t, a, b, u))
    return ws + noise


def estimate_params(times, series, u_s):
    a_min = 0
    b_min = 0
    error2_min = float('inf')
    for a in np.arange(0.1, 1.5, 0.1):
        for b in np.arange(0.1, 1.5, 0.1):
            error2 = 0.0
            for i in range(0, len(u_s)):
                y = generate_sample(times, u_s[i], a, b, False)
                error2 = error2 + sum(map(lambda x: x * x, series[i] - y))
            if (error2 < error2_min):
                error2_min = error2
                a_min = a
                b_min = b
    return a_min, b_min


times = np.arange(0, 5, 0.05)

samples = [
    generate_sample(times, 12, 1, 1),
    generate_sample(times, 8, 1, 1),
    generate_sample(times, 6, 1, 1),
]

u_s = [12, 8, 6]

a, b = estimate_params(times, samples, u_s)
print "Estimated parameters: a = %f, b = %f" % (a, b)

for i in range(0, len(u_s)):
    pp.plot(times, samples[i])
    pp.plot(times, generate_sample(times, u_s[i], a, b, False))

pp.grid(True)
pp.show()