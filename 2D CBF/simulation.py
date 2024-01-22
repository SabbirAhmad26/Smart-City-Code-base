import numpy as np
import matplotlib.pyplot as plt
from map import Map
from robot import LIMO


def main(origin, destination, render_mode, data_plot):
    goal = {'x': destination[0], 'y': destination[1]}
    # obs = [{'x': 1.2, 'y': -0.2, 'r': 0.1}, {'x': 2.4, 'y': 0.2, 'r': 0.1}]
    obs = [{'x': 1.5, 'y': 0.02, 'r': 0.1}]
    limo = LIMO([origin[0], origin[1], 0, 0])
    map = Map(origin, destination, render_mode, data_plot)
    dt = 0.1
    dist2goal = np.sqrt((limo.x - goal['x']) ** 2 + np.sqrt(limo.y - goal['y']) ** 2)
    iter = 0
    # params = [0.1, 0.5, 2.2710451529971425, 0.9151994081810154]
    params = [0.7, 0.6, 2.2710451529971425, 0.9151994081810154]
    while dist2goal > 0.05:
        # Data logging
        limo.data['x'].append(limo.x)
        limo.data['y'].append(limo.y)
        limo.data['psi'].append(limo.psi)
        limo.data['v'].append(limo.v)
        limo.data['acc'].append(limo.acc)
        limo.data['steer'].append(limo.steer)
        limo.data['time'].append(iter * dt)

        limo.acc, limo.steer = limo.control(goal, obs, params)
        limo._move([0, dt], dt)
        map.render([limo], obs)
        dist2goal = np.sqrt((limo.x - goal['x']) ** 2 + (limo.y - goal['y']) ** 2)
        iter += 1

    return limo.data


if __name__ == '__main__':
    origin = (0, 0)
    destiniation = (4, 0)
    data = main(origin, destiniation, render_mode=True, data_plot=True)

