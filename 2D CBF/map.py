import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import time


class Map:
    def __init__(self, origin, destination, render_mode=False, plot_data=False):
        self.lane_width = 4
        self.render_mode = render_mode
        self.origin = origin
        self.destination = destination
        self.plot_data = plot_data

        if self.render_mode:
            plt.style.use('seaborn-v0_8')
            plt.ion()
            self.fig = plt.figure()
            self.ax = self.fig.add_axes([0.1, 0.1, 0.8, 0.8])
        if self.plot_data:
            self.fig1, self.axs = plt.subplots(2, 1)


    def render(self, vehicles, obstacles):
        if self.render_mode:
            if obstacles == []:
                frame = [[self.origin[0] - 1, self.destination[0] + 1],
                         [min(self.destination[1], self.origin[0]) - 1,
                          max(self.destination[1], self.origin[1]) + 1]]
                self.ax.set_xlim(left=frame[0][0], right=frame[0][1])
                self.ax.set_ylim(bottom=frame[1][0], top=frame[1][1])
            else:
                frame = [[self.origin[0] - 1, self.destination[0] + 1],
                         [min(self.destination[1], self.origin[0], obstacles[-1]['y']) - 1,
                          max(self.destination[1], self.origin[1], obstacles[-1]['y']) + 1]]
                self.ax.set_xlim(left=frame[0][0], right=frame[0][1])
                self.ax.set_ylim(bottom=frame[1][0], top=frame[1][1])

            rectangle = patches.Rectangle((self.origin[0], self.origin[1]), 0.1, 0.1, linewidth=1,
                                          edgecolor='green', facecolor='green')
            self.ax.add_patch(rectangle)
            rectangle = patches.Rectangle((self.destination[0], self.destination[1]), 0.1, 0.1,
                                          linewidth=1, edgecolor='green', facecolor='green')
            self.ax.add_patch(rectangle)

            for obs in obstacles:
                Drawing_obs_circle = plt.Circle((obs['x'], obs['y']), obs['r'])
                self.ax.set_aspect(1)
                self.ax.add_artist(Drawing_obs_circle)

            for car in vehicles:
                rotH = np.array([[np.cos(car.states[2]), -np.sin(car.states[2])],
                             [np.sin(car.states[2]), np.cos(car.states[2])]])
                rot_corner_H = rotH.dot(car.corner.transpose()).transpose()
                self.ax.plot(car.states[0] + rot_corner_H[:, 0], car.states[1] + rot_corner_H[:, 1],
                         color='r', linewidth=3.0)
                if self.plot_data:
                    self.axs[1].set_title('CBF')
                    self.axs[1].plot(car.CBF)
                    self.axs[0].set_title('distance')
                    self.axs[0].plot(car.distance)

            # self.axs[0].set_aspect(1)
            # self.axs[1].set_aspect(1)
            self.ax.set_aspect(1)
            self.fig.canvas.draw()
            # time.sleep(0.1)
            self.fig.canvas.flush_events()
            self.ax.clear()
