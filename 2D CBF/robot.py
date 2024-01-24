from scipy.integrate import odeint
import numpy as np
from scipy.integrate import odeint, solve_ivp, ode
import matplotlib.pyplot as plt
from map import Map
import time
import numpy as np
from scipy.optimize import minimize
from cvxopt import matrix
import cvxopt
import cvxopt.solvers




class Robot:
    def __init__(self, states):
        self.states = states
        self.x = states[0]
        self.y = states[1]
        self.psi = states[2]
        self.v = states[3]
        self.steer = 0
        self.acc = 0
        self.umin = -10
        self.umax = 5
        self.steermax = 0.46
        self.steermin = -0.46
        self.vmin = 0
        self.vmax = 0.5
        self.lf = 0.39
        self.lr = 0.39
        self.data = {'x': [], 'y': [], 'psi': [], 'v': [], 'acc': [], 'steer': [], 'time': []}
        self.CBF = []
        self.distance = []


    def _getstates(self):
        return [self.x, self.y, self.v, self.psi]

    def _setstates(self, next_state):
        self.states = next_state[0:4]
        self.x, self.y, self.psi, self.v = next_state[0], next_state[1], next_state[2],next_state[3]

    def _move(self, timespan, teval):
        inputs = [self.acc, self.steer]
        curr_states = self.states
        y0 = np.append(curr_states, inputs)
        sol = solve_ivp(self.dynamics, timespan, y0, method='DOP853', t_eval=[teval], atol=1e-6)
        rt = np.reshape(sol.y[0:len(curr_states)], len(curr_states))
        self._setstates(rt)




class LIMO(Robot):
    def __init__(self, states, model=None):
        super().__init__(states)
        self.length = 0.3
        self.width = 0.2
        self.corner = np.array([[-self.length/2, -self.width/2], [-self.length/2, +self.width/2],
                                [+self.length/2, +self.width/2], [+self.length/2, -self.width/2],
                                 [-self.length/2, -self.width/2]])

    def dynamics(self, t, x):


        dx = [0] * 6
        dx[0] = x[3] * np.cos(x[2])
        dx[1] = x[3] * np.sin(x[2])
        dx[2] = x[3] / self.length * x[5]
        dx[3] = x[4]
        dx[4] = 0
        dx[5] = 0
        return dx

    def control(self, goal, obstacles,params):
        xd = goal['x']
        yd = goal['y']

        A = np.array([[1, 0, 0, 0, 0], [-1, 0, 0, 0, 0], [0, 1, 0, 0, 0], [0, -1, 0, 0, 0]])
        b = np.array([self.umax, -self.umin, self.steermax, -self.steermin])

        x0 = self.x
        y0 = self.y
        v0 = self.v
        psi0 = self.psi

        eps = 10
        psc = 1

        if psi0 < -np.pi:
            psi0 = np.pi


        # Condition 2
        if psi0 > np.pi:
            psi0 = -np.pi


        # Calculate theta_d
        psi_d = np.arctan2(yd - y0, xd - x0)
        psi_dr = psi_d

        # Additional conditions
        if psi0 < 0 and psi0 > -np.pi and psi_d >= np.pi + psi0 and psi_d <= np.pi:
            psi_d = -1.5 * np.pi

        if psi0 > 0 and psi0 < np.pi and psi_d <= -np.pi + psi0 and psi_d >= -np.pi:
            psi_d = 1.5 * np.pi


        V = (psi0 - psi_d)**2
        # print(psi0)
        LfV = eps*10 * V ** 2
        LgV_psi = (v0 * (2 * psi0 - 2 * psi_d)) / (self.length)

        A = np.append(A, [[0, LgV_psi, -1, 0, 0]], axis=0)
        b = np.append(b, [-LfV])


        dist_dst = np.sqrt((x0 - xd) ** 2 + (y0 - yd) ** 2)
        # Calculate vd
        vd = self.vmax
        V_speed = (v0 - vd) ** 2
        LfV_speed = eps*V_speed
        LgV_speed = 2*(v0 - vd)

        A = np.append(A, [[LgV_speed, 0, 0, -1, 0]], axis=0)
        b = np.append(b, [-LfV_speed])

        # B = []
        # LfB = []
        # Lf2B = []
        # Lg2psi = []
        # Lg2acc = []
        for o, obs in enumerate(obstacles):
            k1 = params[2*o + 0]
            k2 = params[2*o + 1]
            xo = obs['x']
            yo = obs['y']
            r = obs['r'] + self.length

            # B = (x0 - xo) ** 2 + (y0 - yo) ** 2 - r ** 2
            # LfB = k1 * ((x0 - xo) ** 2 + (y0 - yo) ** 2 - r ** 2) + v0 * np.cos(psi0) * (
            #             2 * x0 - 2 * xo) + v0 * np.sin(psi0) * (2 * y0 - 2 * yo)
            #
            # Lf2B = k2 * (k1 * ((x0 - xo) ** 2 + (y0 - yo) ** 2 - r ** 2) + v0 * np.cos(psi0) * (2 * x0 - 2 * xo) + v0 * np.sin(psi0) * (
            #             2 * y0 - 2 * yo)) + v0 * np.cos(psi0) * (2 * v0 * np.cos(psi0) + k1 * (2 * x0 - 2 * xo)) + v0 * np.sin(
            #     psi0) * (2 * v0 * np.sin(psi0) + k1 * (2 * y0 - 2 * yo))
            #
            # Lg2psi = (v0 * (v0 * np.cos(psi0) * (2 * y0 - 2 * yo) - v0 * np.sin(psi0) * (2 * x0 - 2 * xo))) / (self.length)
            #
            # Lg2acc = np.cos(psi0)*(2*x0 - 2*xo) + np.sin(psi0)*(2*y0 - 2*yo)


            B = np.sqrt((x0 - xo) ** 2 + (y0 - yo) ** 2) - r
            LfB = (v0 * (x0 * np.cos(psi0) - xo * np.cos(psi0) + y0 * np.sin(psi0) - yo * np.sin(psi0))) / np.sqrt(
                (x0 - xo) ** 2 + (y0 - yo) ** 2) - \
                   k1 * (r - np.sqrt((x0 - xo) ** 2 + (y0 - yo) ** 2))

            Lf2B = v0 * np.cos(psi0) * ((k1 * (x0 - xo)) / np.sqrt((x0 - xo) ** 2 + (y0 - yo) ** 2) + (
                        v0 * np.cos(psi0)) / np.sqrt((x0 - xo) ** 2 +
                                                     (y0 - yo) ** 2) - (v0 * (2 * x0 - 2 * xo) * (
                        x0 * np.cos(psi0) - xo * np.cos(psi0) + y0 * np.sin(psi0) - yo * np.sin(psi0))) / (
                                                     2 * ((x0 - xo) ** 2 +
                                                          (y0 - yo) ** 2) ** (3 / 2))) - k2 * (
                                k1 * (r - np.sqrt((x0 - xo) ** 2 + (y0 - yo) ** 2)) - (
                                    v0 * (x0 * np.cos(psi0) - xo * np.cos(psi0) +
                                          y0 * np.sin(psi0) - yo * np.sin(psi0))) / np.sqrt(
                            (x0 - xo) ** 2 + (y0 - yo) ** 2)) + v0 * np.sin(psi0) * (
                                (k1 * (y0 - yo)) / np.sqrt((x0 - xo) ** 2 +
                                                                   (y0 - yo) ** 2) + (v0 * np.sin(psi0)) / np.sqrt(
                            (x0 - xo) ** 2 + (y0 - yo) ** 2) - (v0 * (2 * y0 - 2 * yo) * (x0 * np.cos(psi0) -
                                                                                             xo * np.cos(
                                    psi0) + y0 * np.sin(psi0) - yo * np.sin(psi0))) / (
                                            2 * ((x0 - xo) ** 2 + (y0 - yo) ** 2) ** (3 / 2)))

            Lg2acc = (x0 * np.cos(psi0) - xo * np.cos(psi0) + y0 * np.sin(psi0) - yo * np.sin(psi0)) / np.sqrt(
                (x0 - xo) ** 2 + (y0 - yo) ** 2)



            Lg2psi = (v0 ** 2 * (y0 * np.cos(psi0) - yo * np.cos(psi0) - x0 * np.sin(psi0) + xo * np.sin(psi0))) / (
                        (self.length) * np.sqrt((x0 - xo) ** 2 + (y0 - yo) ** 2))


            A = np.append(A, [[-Lg2acc, -Lg2psi, 0, 0, 0]], axis=0)
            b = np.append(b, [Lf2B])




        # k1 = 1
        # k2 = 1
        # V = (y0 - 1.9)
        # LfV = k1 * (y0 - 1.9) ** 2 + v0 * np.sin(psi0) * (2 * y0 - 2 * 1.9)
        # L2fV = k2 * (k1 * (y0 - 1.9) ** 2 + v0 * np.sin(psi0) * (2 * y0 - 2 * 1.9)) + v0 * np.sin(psi0) * (
        #             2 * v0 * np.sin(psi0) + k1 * (2 * y0 - 2 * 1.9))
        # LfLg_psi = (v0**2*np.cos(psi0)*(2*y0 - 2*1.9))/(self.lf + self.lr)
        # Psi2_Lg_u = np.sin(psi0)*(2*y0 - 2*1.9)
        #
        # A = np.append(A, [[Psi2_Lg_u, LfLg_psi, 0,  0, -1]], axis=0)
        # b = np.append(b, [-L2fV])



        # H = np.array([[1.0, 0, 0, 0],[0, 0.0001, 0, 0],[0, 0, 2*psc, 0],[0, 0, 0, 10*psc]])
        H = np.array([[1.0, 0, 0, 0, 0], [0, 1.0, 0, 0, 0], [0, 0, 2 * psc, 0, 0], [0, 0, 0, 200 * psc, 0],
                      [0, 0, 0, 0, 1 * psc]])
        f = np.zeros((5, 1))



        P = matrix(H)
        q = matrix(f)
        G = matrix(A)  # cvxopt uses <= inequality, so multiply by -1
        h = matrix(b)

        try :
            options = {'show_progress': False}
            Solution = cvxopt.solvers.qp(P, q, G, h, options=options)
            if Solution["status"] == 'optimal':
                u = Solution['x']
            else:
                u = Solution['x']
                if u[0] > self.umax:
                    u[0] = self.umax
                else:
                    u[0] = self.umin
            if obstacles != []:
                self.CBF.append(Lf2B + Lg2acc*u[0] + Lg2psi*u[1])
                self.distance.append(B)
        except:
            # Handle the case when no optimal solution is found
            u = matrix([self.umin/2, 0.0])  # [-cd * m * g, 0]

        return u[0], u[1]







