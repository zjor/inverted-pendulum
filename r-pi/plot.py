"""
Plots tab-separated data
First column is considered as a timeseries
"""

import sys
import matplotlib.pyplot as pp
import numpy as np

def plot(data):
    series = np.array(data).T
    for i in range(1, len(series)):
        pp.plot(series[0], series[i])
    pp.show()

def parse(filename):
    data = []
    with open(filename) as f:
        for line in f.readlines():
            values = line.strip().split()
            data.append(map(lambda x: float(x), values))
    return data


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print "Usage: python plot.py <filename>"
    else:
        data = parse(sys.argv[1])
        plot(data)