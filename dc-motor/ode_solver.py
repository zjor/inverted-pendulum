import numpy as np


def integrate_euler(state, step, t, dt, dydx_func):
    """
    First-order solver. Uses Euler's method
    :param state:
    :param step:
    :param t:
    :param dt:
    :param dydx_func:
    :return:
    """
    k1 = dydx_func(state, step, t, dt)
    return [v + k1_ * dt for v, k1_ in zip(state, k1)]


def integrate_heuns(state, step, t, dt, dydx_func):
    """
    Second-order Heun's method.
    Source: https://en.wikipedia.org/wiki/Heun%27s_method
    :param state:
    :param step:
    :param t:
    :param dt:
    :param dydx_func:
    :return:
    """
    k1 = dydx_func(state, step, t, dt)
    k2 = dydx_func([v + d * dt for v, d in zip(state, k1)], step, t, dt)
    return [v + (k1_ + k2_) * dt / 2 for v, k1_, k2_ in zip(state, k1, k2)]


def integrate_rk4(state, step, t, dt, dydx_func):
    """
    Fourth-order Runge-Kutta method.
    Source: https://www.geeksforgeeks.org/runge-kutta-4th-order-method-solve-differential-equation/
    :param step:
    :param state:
    :param t:
    :param dt:
    :param dydx_func:
    :return:
    """
    k1 = dydx_func(state, step, t, dt)
    k2 = dydx_func([v + d * dt / 2 for v, d in zip(state, k1)], step, t, dt)
    k3 = dydx_func([v + d * dt / 2 for v, d in zip(state, k2)], step, t, dt)
    k4 = dydx_func([v + d * dt for v, d in zip(state, k3)], step, t, dt)
    return [v + (k1_ + 2 * k2_ + 2 * k3_ + k4_) * dt / 6 for v, k1_, k2_, k3_, k4_ in zip(state, k1, k2, k3, k4)]


def derivatives_circle(state, step, t, dt):
    x, v = state
    return [v, -x]


def solve(initial_state, times, integrate_func, derivative_func):
    """
    Solves the initial-value problem of the first order ODEs
    :param initial_state: initial state
    :param times: a sequence of time points for which to solve
    :param integrate_func: calculates the next state
    :param derivative_func: computes derivatives of each state component
    :return:
    """
    dt = times[1] - times[0]
    states = [initial_state]
    for step, t in enumerate(times):
        states.append(integrate_func(states[-1], step, t, dt, derivative_func))
    return np.array(states)
