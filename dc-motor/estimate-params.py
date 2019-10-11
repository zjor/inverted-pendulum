import re
import csv
import glob
import numpy as np
import matplotlib.pyplot as pp

from math import pi

def map_to_float(a):
    return map(lambda x: float(x), a)

def get_filenames_and_pwm():
    result = []
    for filename in glob.glob("./data/pwm-*.csv"):
        pwm = int(re.search('(\\d+)', filename).group(1))
        result.append((pwm, filename))
    return result

def read_params(pwm):
    filename = "./data/pwm-%d.csv" % pwm
    data = []
    with open(filename) as f:
        reader = csv.reader(f)
        for row in reader:
            data.append(map_to_float(row))
    data = np.array(data)
    rows, cols = data.shape
    ext = np.zeros((rows, cols + 1))
    ext[:,:-1] = data
    ext[:, 3] = ext[:, 3] * 2.0 * pi / 2400.0 * 0.00573
    for i in range(1, rows):
        dt = (ext[i, 2] - ext[i - 1, 2]) / 1000.0
        ext[i, -1] = (ext[i, 3] - ext[i - 1, 3]) / dt
    return ext

def read_all():
    result = {}
    pwms = [100, 150, 200, 230]
    for pwm in pwms: #get_filenames_and_pwm():
        data = read_params(pwm)
        result[pwm] = data
    return result

def plot_velocity(data):
    for value in data.values():
        line, = pp.plot(value[:,2], value[:, 4], label=value[0, 1])
    pp.legend()
    pp.show()

def get_set_speed(data):
    s = data[-1, 2] + data[-2, 2] + data[-3, 2]
    return s / 3

def get_set_speed_for_all(data):
    result = []
    for i in data.items():
        result.append((i[0], get_set_speed(i[1])))
    return np.array(sorted(result, key=lambda x: x[0]))

all_data = read_all()

# vs = get_set_speed_for_all(all_data)
# pp.plot(vs[:, 0], vs[:, 1])
# pp.show()

plot_velocity(all_data)    
# TODO:
# - plot set velocity, check linearity
# - fit a, b, c parameters