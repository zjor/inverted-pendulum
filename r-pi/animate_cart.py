import numpy as np
import matplotlib.pyplot as pp
import scipy.integrate as integrate
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

from math import pi, trunc
from numpy import sin, cos


def animate_cart(cart_x, thetas, L, dt):

    pxs = L * sin(thetas) + cart_x
    pys = L * cos(thetas)


    fig = pp.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-1, 2))
    ax.set_aspect('equal')
    ax.grid()

    patch = ax.add_patch(Rectangle((0, 0), 0, 0, linewidth=1, edgecolor='k', facecolor='g'))

    line, = ax.plot([], [], 'o-', lw=2)
    time_template = 'time = %.1fs'
    time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

    cart_width = 0.3
    cart_height = 0.2

    def init():
        line.set_data([], [])
        time_text.set_text('')
        patch.set_xy((-cart_width/2, -cart_height/2))
        patch.set_width(cart_width)
        patch.set_height(cart_height)
        return line, time_text, patch


    def animate(i):
        thisx = [cart_x[i], pxs[i]]
        thisy = [0, pys[i]]

        line.set_data(thisx, thisy)
        time_text.set_text(time_template % (i*dt))
        patch.set_x(cart_x[i] - cart_width/2)
        return line, time_text, patch

    ani = animation.FuncAnimation(fig, animate, np.arange(1, len(thetas)),
                                  interval=25, blit=True, init_func=init)

    pp.show()

def load_data(filename):
    thetas = []
    cart_x = []
    last_t = 0.0
    dts = []
    with open(filename) as f:
        for line in f.readlines():
            row = map(lambda x: float(x), line.strip().split())
            cart_x.append(row[1])
            thetas.append(row[3])
            if last_t != 0.0:
                dts.append(row[0] - last_t)
            last_t = row[0]

    return (cart_x, thetas, sum(dts) / len(dts))

if __name__ == "__main__":
    import sys
    cart_x, thetas, dt = load_data(sys.argv[1])
    animate_cart(cart_x, thetas, 1.0, dt)