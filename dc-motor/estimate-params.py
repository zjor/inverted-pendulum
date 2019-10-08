import re
import csv
import glob
import numpy as np
import matplotlib.pyplot as pp

def map_to_float(a):
    return map(lambda x: float(x), a)

def normalize_time(data):
    t0 = data[0, 0]
    for row in data:
        row[0] = row[0] - t0
    return data

def get_filenames_and_pwm():
    result = []
    for filename in glob.glob("./data/dc-params-pwm-*.csv"):
        pwm = int(re.search('(\\d+)', filename).group(1))
        result.append((pwm, filename))
    return result

def read_params(pwm):
    filename = "./data/dc-params-pwm-%d.csv" % pwm
    data = []
    with open(filename) as f:
        reader = csv.reader(f)
        for row in reader:
            data.append(map_to_float(row))
    return np.array(data)

def read_all():
    result = {}
    for datafile in get_filenames_and_pwm():
        data = read_params(datafile[0])
        result[datafile[0]] = data
    return result

def plot_all(data):
    for value in data.values():
        line, = pp.plot(value[:,0], value[:, 2], label=value[0, 1])
    pp.legend()
    pp.show()

plot_all(read_all())    
# TODO:
# - plot all data on a single plot
# - plot set velocity, check linearity
# - fit a, b, c parameters