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
    """
        [i, pwm, time, x, v]
    """
    if pwm > 0:
        filename = "./data/pwm-%d.csv" % pwm
    else:
        filename = "./data/pwm-rev-%d.csv" % (-pwm)
    data = []
    with open(filename) as f:
        reader = csv.reader(f)
        for row in reader:
            data.append(map_to_float(row))
    data = np.array(data)
    rows, cols = data.shape
    ext = np.zeros((rows, cols + 1))
    ext[:,:-1] = data    
    for i in range(1, rows):
        dt = (ext[i, 2] - ext[i - 1, 2]) / 1000000.0
        ext[i, -1] = (ext[i, 3] - ext[i - 1, 3]) / dt
    return ext

def read_all():
    result = {}
    pwms = [-250, -240, -230, -210, -200, -170, -150, -120, -100, -90, -70, -60, 60, 70, 80, 90, 100, 150, 200, 230]
    for pwm in pwms: #get_filenames_and_pwm():
        data = read_params(pwm)
        result[pwm] = data
    return result

def plot_velocity(data):
    for value in data.values():
        line, = pp.plot(value[:,2], value[:, 4], label=value[0, 1])
    pp.legend()
    pp.grid(True)
    pp.show()

def plot_set_velocity(data):
    set_vs = np.zeros((len(data), 2))
    for i, value in enumerate(data.values()):
        velocities = value[-10:-1, 4]
        set_vs[i, 0] = value[0, 1]
        set_vs[i, 1] = sum(velocities) / len(velocities)

    pp.plot(set_vs[:, 0], set_vs[:, 1], 'o')
    pp.grid(True)
    pp.show()


all_data = read_all()

# plot_velocity(all_data)
plot_set_velocity(all_data)

