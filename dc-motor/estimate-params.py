import csv
import glob
import numpy as np
import matplotlib.pyplot as pp

def map_to_float(a):
    return map(lambda x: float(x), a)

def trim_zeroes(data):
    pass

def normalize_time(data):
    pass

def read_params(pwm):
    filename = "./data/dc-params-pwm-%d.csv" % pwm
    data = []
    with open(filename) as f:
        reader = csv.reader(f)
        for row in reader:
            data.append(map_to_float(row))
    return np.array(data)

# params = read_params(45)

# pp.plot(params[:, 0], params[:, 2])
# pp.show()

print glob.glob("./data/dc-params-pwm-*.csv")

# TODO:
# - plot all data on a single plot
# - plot set velocity, check linearity
# - fit a, b, c parameters