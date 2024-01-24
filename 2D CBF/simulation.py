import numpy as np
import matplotlib.pyplot as plt
from map import Map
from robot import LIMO


def main(origin, destination, obs, params, render_mode, data_plot):
    goal = {'x': destination[0], 'y': destination[1]}
    obs = [{'x': obs[0], 'y': obs[1], 'r': obs[2]}]
    limo = LIMO([origin[0], origin[1], 0, 0])
    map = Map(origin, destination, render_mode, data_plot)
    dt = 0.1
    dist2goal = np.sqrt((limo.x - goal['x']) ** 2 + (limo.y - goal['y']) ** 2)
    iter = 0

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
    origin = (1.7, 1.9)  # where robot starts
    destiniation = (6.4, 1.9)  # where robot ends
    obs = (4.0, 1.88, 0.17)  # obstacle (x, y, radius)
    params = [0.8, 0.5]  # cbf class-k parameters
    data = main(origin, destiniation, obs, params, render_mode=True, data_plot=True)
    stop = 1

