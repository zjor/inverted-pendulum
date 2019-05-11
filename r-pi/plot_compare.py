"""
t, u, v, x, x_real
"""

import sys
import matplotlib.pyplot as pp
import numpy as np

def plot(data):
    series = np.array(data).T
    
    pp.subplot(311)
    pp.plot(series[0], series[3])
    pp.plot(series[0], series[4])
    pp.grid(True)

    pp.subplot(312)
    pp.plot(series[0], series[2])
    pp.plot(series[0], series[5])
    pp.grid(True)

    pp.subplot(313)
    pp.plot(series[0], series[1])
    pp.plot(series[0], series[6])
    pp.grid(True)

    pp.show()


def enrich(data):
    data[0].extend([.0, .0])
    for i in range(1, len(data) - 1):
        h = data[i][0] - data[i - 1][0]
        v = (data[i][4] - data[i - 1][4]) / h
        a = (data[i - 1][4] - 2.0 * data[i][4] + data[i + 1][4]) / (h * h)
        data[i].extend([v, a])

    for i in range(2, len(data) - 2):
        h = data[i][0] - data[i - 1][0]
        a = (-data[i + 2][4] + 16.0 * data[i + 1][4] - 30.0 * data[i][4] - data[i - 2][4] + 16.0 * data[i - 1][4]) / (12.0 * h * h)
        data[i][6] = a

    data[-1].extend([.0, .0])    


def parse(filename):
    data = []
    with open(filename) as f:
        for line in f.readlines():
            values = line.strip().split()
            data.append(map(lambda x: float(x), values))
    return data


if __name__ == "__main__":
    data = parse("data.txt")
    enrich(data)
    plot(data)