import numpy as np
import matplotlib.pyplot as plt
import time


class Map:
    def __init__(self, render_mode=False):
        self.lane_width = 4
        self.render_mode = render_mode


        if self.render_mode:
            #plt.style.use('seaborn-v0_8')
            plt.ion()
            self.fig = plt.figure()
            self.ax = self.fig.add_axes([0.1, 0.1, 0.8, 0.8])
            self.fig1 = plt.figure()
            # self.ax1 = self.fig1.add_axes([0.1, 0.1, 0.8, 0.8])


    def render(self, vehicles):
        for c, car in enumerate(vehicles):
            xpoints = np.array([-10, 2000])
            ypoints1 = np.array([-3, -3])
            ypoints2 = np.array([3, 3])
            ypoints3 = np.array([9, 9])
            self.ax.plot(xpoints, ypoints1, 'k')
            self.ax.plot(xpoints, ypoints2, 'y--')
            self.ax.plot(xpoints, ypoints3, 'k')
            self.ax.axis([-80 + car.states[0], 80 + car.states[0], -16, 16])
            rotH = np.array([[np.cos(car.states[2]), -np.sin(car.states[2])],
                         [np.sin(car.states[2]), np.cos(car.states[2])]])
            rot_corner_H = rotH.dot(car.corner.transpose()).transpose()
            self.ax.plot(car.states[0] + rot_corner_H[:, 0], car.states[1] + rot_corner_H[:, 1],
                     color=car.color, linewidth=3.0)

            if c < 2:
                if c == 0:
                    aH = 0.6
                    bH = 0.1
                else:
                    aH = 0.6
                    bH = 0.2
                delta = 0.01
                a = aH * car.v + delta  # radius on the x-axis
                b = bH * car.v + delta  # radius on the y-axis
                t_rot = car.phi  # rotation angle

                t = np.linspace(0, 2 * np.pi, 100)
                Ell = np.array([a * np.cos(t), b * np.sin(t)])
                # u,v removed to keep the same center location
                R_rot = np.array([[np.cos(t_rot), -np.sin(t_rot)], [np.sin(t_rot), np.cos(t_rot)]])
                # 2-D rotation matrix
                Ell_rot = np.zeros((2, Ell.shape[1]))
                for i in range(Ell.shape[1]):
                    Ell_rot[:, i] = np.dot(R_rot, Ell[:, i])

                self.ax.plot(car.x + Ell_rot[0, :], car.y + Ell_rot[1, :], 'darkorange')  # rotated ellipse


        # self.ax1.plot(vehicles[1].CBF_with_CAV1)

        self.ax.set_aspect(1)
        self.fig.canvas.draw()
        # time.sleep(0.1)
        self.fig.canvas.flush_events()
        self.ax.clear()
