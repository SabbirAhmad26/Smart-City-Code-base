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
import random

class Vehicle:
    def __init__(self, states):
        self.states = states
        self.x = states[0]
        self.y = states[1]
        self.phi = states[2]
        self.v = states[3]
        self.acc = 0
        self.steer = 0
        self.umin = -7
        self.umax = 3.3
        self.steermax = np.pi/4
        self.steermin = -np.pi/4
        self.CBF_with_CAV1 = []
        self.vmin = 15
        self.vmax = 35
        self.length = 5.0
        self.width = 2.0
        self.h = [0, 0, 0, 0]
        self.corner = np.array([[-self.length/2, -self.width/2], [-self.length/2, +self.width/2],
                                [+self.length/2, +self.width/2], [+self.length/2, -self.width/2], [-self.length/2, -self.width/2]])


    def _getstates(self):
        return [self.x, self.y, self.phi, self.v]

    def _setstates(self, next_state):
        self.states = next_state[0:4]
        self.x, self.y, self.phi, self.v = next_state[0:4]

    def _move(self, timespan, teval, flag):
        inputs = [self.acc, self.steer]
        curr_states = [self.x, self.y, self.phi, self.v]
        y0 = np.append(curr_states, inputs)
        if flag:
            eps1 = 0 * 1.4 * (0.5 - np.random.rand())  # [-0.2, 0.2]
            eps2 = 0 * 1 * (0.5 - np.random.rand())  # [-0.02, 0.02]
            eps3 = 0 * 1 * (0.5 - np.random.rand())  # [-0.01, 0.01]
            eps4 = 0 * 1.4 * (0.5 - np.random.rand())  # [-0.2, 0.2]
            sigma1 = 1 + 0 * 0.01 *0.2 * (0.5 - np.random.rand())  # [0.9, 1.1]
            sigma2 = 1 + 0 * 0.2 * (0.5 - np.random.rand())  # [0.9, 1.1]
            other_vars = [sigma1, sigma2, eps1, eps2, eps3, eps4]
            y0 = np.append(y0, other_vars)

        sol = solve_ivp(self.dynamics, timespan, y0, method='DOP853', t_eval=[teval], atol=1e-6)
        rt = np.reshape(sol.y[0:len(curr_states)], len(curr_states))
        self._setstates(rt)




class HDV(Vehicle):
    def __init__(self, states, model=None):
        super().__init__(states)
        self.acc_manual = 0
        self.steer_manual = 0
        self.color = 'red'

    def dynamics(self, t, x):


        dx = [0] * 12
        dx[0] = x[3] * np.cos(x[2]) * x[6] - x[3] * np.sin(x[2]) * x[5] + x[8]
        dx[1] = x[3] * np.sin(x[2]) * x[7] + x[3] * np.cos(x[2]) * x[5] + x[9]
        dx[2] = (x[3] * x[5]) / self.length + x[10]
        dx[3] = x[4] + x[11]
        dx[4] = 0 #acc
        dx[5] = 0 #phiH
        dx[6] = 0
        dx[7] = 0
        dx[8] = 0
        dx[9] = 0
        dx[10] = 0
        dx[11] = 0
        return dx

    def control(self):

        uH_ref = self.acc_manual
        phi_H_ref = self.steer_manual
        lanewidth = 4
        # uH_ref = self.acc
        # phi_H_ref = self.steer
        k9 = 1
        k8 = 1
        H_hdv = np.eye(2)
        F_hdv = np.array([[-2 * uH_ref], [-2 * phi_H_ref]])
        A_hdv = np.array([[0, -self.v * np.cos(self.phi)], [0, self.v * np.cos(self.phi)], [1, 0]
                          ,[-1, 0], [0, 1], [0, -1]])
        b_hdv = np.array([[self.v * np.sin(self.phi) + k8 * (self.y + lanewidth/ 2)],
        [-self.v * np.sin(self.phi) + k9 * (3 * lanewidth / 2 - self.y)], [self.umax], [-self.umin],[self.steermax], [-self.steermin] ])

        # A_hdv = np.array([[1.0, 0],[-1.0, 0], [0, 1.0], [0, -1.0]])
        # b_hdv = np.array([[self.umax], [-self.umin], [self.steermax], [-self.steermin]])
        # F_hdv = np.zeros((2, 1))


        P = matrix(H_hdv)
        q = matrix(F_hdv)
        G = matrix(A_hdv)  # cvxopt uses <= inequality, so multiply by -1
        h = matrix(b_hdv)

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

        return u[0], u[1]

class CAV(Vehicle):
    def __init__(self, states, model=None):
        super().__init__(states)
        self.color = 'green'

    def dynamics(self, t, x):
        self.length = 4
        dx = [0] * 6
        dx[0] = x[3] * np.cos(x[2]) - x[3] * np.sin(x[2]) * x[5]
        dx[1] = x[3] * np.sin(x[2]) + x[3] * np.cos(x[2]) * x[5]
        dx[2] = x[3] / self.length * x[5]
        dx[3] = x[4]
        dx[4] = 0
        dx[5] = 0

        return dx

    def control(self, xU, x1, u1, phi_1, yH, xH, uH, phi_H, xH_previous):

        def ellipse_safety_1(X_1):
            aH = 0.6
            bH = 0.2
            c1 = X_1[7] ** 2 - (X_1[4] - X_1[0] - X_1[8]) ** 2 / (aH ** 2) - (X_1[5] - X_1[1] - X_1[9]) ** 2 / (
                        bH ** 2)
            return c1

        def ellipse_safety_2(X2):
            aC = 0.6
            bC = 0.1
            c2 = X2[7] ** 2 - (X2[0] - X2[4]) ** 2 / aC ** 2 - (X2[1] - X2[5]) ** 2 / bC ** 2
            return c2

        def ellipse_safety_3(X3):
            aH = 0.6
            bH = 0.1
            c3 = X3[7] ** 2 - (X3[4] - X3[0] - X3[8]) ** 2 / (aH ** 2) - (X3[5] - X3[1] - X3[9]) ** 2 / (bH ** 2)
            return c3

        def ellipse_safety_U(X):
            aC = 0.6
            bC = 0.1
            cU = X[7] ** 2 - (X[0] - X[4]) ** 2 / (aC ** 2) - (X[1] - X[5]) ** 2 / (bC ** 2)
            return cU

        self.length = 4
        lanewidth = 4

        v_des = 30
        self.vmin = 15
        self.vmax = 35

        aH = 0.6
        aC = 0.6
        bH = 0.2
        bC = 0.1
        dt = 0.05
        vU = 20

        s_xc = 0.01
        s_yc = 0.005
        s_vc = 1
        s_thetaC = 0.01
        s_xH = 0.01
        s_yH = 0.005
        s_vH = 1
        s_thetaH = 0.01
        s_x1 = 0.01
        s_y1 = 0.005
        s_v1 = 1
        s_theta1 = 0.01
        s_xU = 0.01
        s_yU = 0.005
        s_vU = 1
        s_thetaU = 0.01
        ex_b = 0.2
        ey_b = 0.1
        ev_b = 1
        ex_dot_b = 0.5
        ey_dot_b = 0.2
        ev_dot_b = 1

        k1 = 1
        k2 = 1
        k3 = 1
        k4 = 10
        k5 = 1
        k6 = 1
        k7 = 1
        k8 = 1
        k9 = 1
        k10 = 1
        k11 = 1
        k12 = 1
        k13 = 1
        kU = 10

        vareps1 = 1
        vareps2 = 1
        vareps3 = 1
        vareps6 = 1
        if not any(np.isnan(x) for x in xH_previous):
            xH_dot = (xH[0] - xH_previous[0]) / dt
            ex_dot = xH_dot - (xH[3] * np.cos(xH[2]) + self.h[0])
            yH_dot = (xH[1] - xH_previous[1]) / dt
            ey_dot = yH_dot - (xH[3] * np.sin(xH[2]) + self.h[1])
            thetaH_dot = (xH[2] - xH_previous[2])/dt
            etheta_dot = thetaH_dot - (xH[3]/self.length + self.h[2])
            vH_dot = (xH[3] - xH_previous[3])/dt
            ev_dot = vH_dot - self.h[3]
        else:
            ex_dot,ey_dot,etheta_dot,ev_dot = [0,0,0,0]

        # ev_dot = uH + eps4 - self.h[3]
        #ex_dot = (xH[3]*np.cos(xH[2])*sigma1 - xH[3]*np.sin(xH[2])*phi_H + eps1) - (xH[1]*np.cos(xH[2]) + self.h[0])
        #ey_dot = (xH[3]*np.sin(xH[2])*sigma2 + xH[3]*np.cos(xH[2])*phi_H + eps2) - (xH[1]*np.sin(xH[2]) + self.h[1])
        #etheta_dot = (xH[3]/self.length*xH[2] + eps3) - (xH[3]/self.length + self.h[2])


        xH_tk = yH[0]
        yH_tk = yH[1]
        thetaH_tk = yH[2]
        vH_tk = yH[3]



        xC = self.states
        xC_tk = self.x
        yC_tk = self.y
        thetaC_tk = self.phi
        vC_tk = self.v


        x1_tk = x1[0]
        y1_tk = x1[1]
        theta1_tk = x1[2]
        v1_tk = x1[3]


        xU_tk = xU[0]
        yU_tk = xU[1]
        thetaU_tk = xU[2]
        vU_tk = xU[3]
        uC = self.acc
        phi_C = self.steer


        A = np.array([[1, 0, 0, 0, 0, 0, 0, 0], [-1, 0, 0, 0, 0, 0, 0, 0],[0, 1, 0, 0, 0, 0, 0, 0], [0, -1, 0, 0, 0, 0, 0, 0]])
        b = np.array([self.umax, -self.umin, self.umax, -self.umin])

        A = np.append(A, [[0, 0, 1, 0, 0, 0, 0, 0],[0, 0, 0, 1, 0, 0, 0, 0]], axis=0)
        b = np.append(b, [self.steermax, self.steermax])

        A = np.append(A, [[0, 0, -1, 0, 0, 0, 0, 0], [0, 0, 0, -1, 0, 0, 0, 0]], axis=0)
        b = np.append(b, [-self.steermin, -self.steermin])


        LfB4 = xC[3] * np.sin(xC[2])
        LgB4 = [0, 0, -xC[3] * np.cos(xC[2]), 0, 0, 0, 0, 0]
        # classK4 = k4 * (xC[1] + lanewidth / 2 - 1)
        classK4 = k4 * (xC[1] + 2)

        A = np.append(A, [LgB4], axis=0)
        b = np.append(b, [LfB4 + classK4])


        LfB5 = -xC[3] * np.sin(xC[2])
        LgB5 = [0, 0, xC[3] * np.cos(xC[2]), 0, 0, 0, 0, 0]
        #classK5 = k5 * (3 / 2 * lanewidth - 1 - xC[1])
        classK5 = k5 * (8 - xC[1])

        A = np.append(A, [LgB5], axis=0)
        b = np.append(b, [LfB5 + classK5])


        # Define the functions LfB6, LgB6, classK6
        LfB6 = x1[3] * np.sin(x1[2])
        LgB6 = [0, 0, 0, -x1[3] * np.cos(x1[2]),0 ,0 ,0, 0]
        #classK6 = k6 * (x1[1] - lanewidth / 2 - 1)
        classK6 = k6 * (x1[1] - 4)

        A = np.append(A, [LgB6], axis=0)
        b = np.append(b, [LfB6 + classK6])


        # Define the functions LfB7, LgB7, classK7
        LfB7 = -x1[3] * np.sin(x1[2])
        LgB7 = [0, 0, 0, x1[3] * np.cos(x1[2]), 0, 0, 0, 0]
        # classK7 = k7 * (3 / 2 * lanewidth - 1 - x1[1])
        classK7 = k6 * (8 - x1[1])

        A = np.append(A, [LgB7], axis=0)
        b = np.append(b, [LfB7 + classK7])

        # Define the functions LfB10, LgB10, classK10

        LfB10 = 0
        LgB10 = [0, -1, 0, 0, 0, 0, 0, 0]
        classK10 = k10 * (x1[3] - self.vmin)

        A = np.append(A, [LgB10], axis=0)
        b = np.append(b, [LfB10 + classK10])

        # Define the functions LfB11, LgB11, classK11
        LfB11 = 0
        LgB11 = [0, 1, 0, 0, 0, 0, 0, 0]
        classK11 = k11 * (self.vmax - x1[3])

        A = np.append(A, [LgB11], axis=0)
        b = np.append(b, [LfB11 + classK11])

        # Define the functions LfB12, LgB12, classK12
        LfB12 = 0
        LgB12 = [-1, 0, 0, 0, 0, 0, 0, 0]
        classK12 = k12 * (xC[3] - self.vmin)

        A = np.append(A, [LgB12], axis=0)
        b = np.append(b, [LfB12 + classK12])

        LfB13 = 0
        LgB13 = [1, 0, 0, 0, 0, 0, 0, 0]
        classK13 = k13 * (self.vmax - xC[3])
        A = np.append(A, [LgB13], axis=0)
        b = np.append(b, [LfB13 + classK13])

        # safety with truck    # xU,yU,theta_U,vU,xC,yC,theta_C,vC
        def objective0(X):
            return 2 * (X[0] - X[4]) / (aC ** 2) * (xU[3] - X[7] * np.cos(X[6])) \
                + 2 * (X[1] - X[5]) / (bC ** 2) * (-X[7] * np.sin(X[6])) \
                + kU * ((X[0] - X[4]) ** 2 / (aC ** 2) + (X[1] - X[5]) ** 2 / (bC ** 2) - X[7] ** 2)

        x0_U = [xU_tk, yU_tk, thetaU_tk, vU_tk, xC_tk, yC_tk, thetaC_tk, vC_tk]
        lb_U = [xU_tk - s_xU, yU_tk - s_yU, thetaU_tk - s_thetaU, vU_tk - s_vU, xC_tk - s_xc, yC_tk - s_yc,
                thetaC_tk - s_thetaC, vC_tk - s_vc]
        ub_U = [xU_tk + s_xU, yU_tk + s_yU, thetaU_tk + s_thetaU, vU_tk + s_vU, xC_tk + s_xc, yC_tk + s_yc,
                thetaC_tk + s_thetaC, vC_tk + s_vc]

        L_fU_optimization = minimize(
            fun=lambda x: objective0(x),
            x0=x0_U,
            constraints={'type': 'ineq', 'fun': lambda x: ellipse_safety_U(x)},
            bounds=[(lb_U[0], ub_U[0]), (lb_U[1], ub_U[1]), (lb_U[2], ub_U[2]), (lb_U[3], ub_U[3]),
                    (lb_U[4], ub_U[4]),
                    (lb_U[5], ub_U[5]), (lb_U[6], ub_U[6]), (lb_U[7], ub_U[7])]
        )

        L_fbU = L_fU_optimization["fun"]

        if phi_C >= 0:
            constant = 1
            def objective1(X):
                return 2 * (X[0] - X[4]) / (aC ** 2) * (X[7] * np.sin(X[6])) - 2 * (X[1] - X[5]) / (bC ** 2) * (
                        X[7] * np.cos(X[6]))
        else:
            constant = -1
            def objective1(X):
                return -2 * (X[0] - X[4]) / (aC ** 2) * (X[7] * np.sin(X[6])) + 2 * (X[1] - X[5]) / (bC ** 2) \
                    * (X[7] * np.cos(X[6]))

        L_gU2_optimization = minimize(
            fun=lambda x: objective1(x),
            x0=x0_U,
            constraints={'type': 'ineq', 'fun': lambda x: ellipse_safety_U(x)},
            bounds=[(lb_U[0], ub_U[0]), (lb_U[1], ub_U[1]), (lb_U[2], ub_U[2]), (lb_U[3], ub_U[3]),
                    (lb_U[4], ub_U[4]),
                    (lb_U[5], ub_U[5]), (lb_U[6], ub_U[6]), (lb_U[7], ub_U[7])]
        )
        L_gbU2 = constant * L_gU2_optimization["fun"]

        if uC >= 0:
            constant = 1
            def objective2(X):
                return -2 * X[7]
        else:
            constant = -1
            def objective2(X):
                return 2 * X[7]

        L_gU3_optimization = minimize(
            fun=lambda x: objective2(x),
            x0=x0_U,
            constraints={'type': 'ineq', 'fun': lambda x: ellipse_safety_U(x)},
            bounds=[(lb_U[0], ub_U[0]), (lb_U[1], ub_U[1]), (lb_U[2], ub_U[2]), (lb_U[3], ub_U[3]),
                    (lb_U[4], ub_U[4]),
                    (lb_U[5], ub_U[5]), (lb_U[6], ub_U[6]), (lb_U[7], ub_U[7])]
        )

        L_gbU3 = constant * L_gU3_optimization["fun"]
        LfBU = L_fbU
        LgBU = np.array([-L_gbU3, 0, -L_gbU2, 0, 0, 0, 0, 0])
        classKU = 0

        A = np.append(A, [LgBU], axis=0)
        b = np.append(b, [LfBU + classKU])
        # end


        # safety with CAV 1

        def objective01(X2):
            return 2*(X2[0]-X2[4])/(aC**2)*(X2[3]*np.cos(X2[2])-X2[7]*np.cos(X2[6])) \
                + 2*(X2[1]-X2[5])/(bC**2)*(X2[3]*np.sin(X2[2])-X2[7]*np.sin(X2[6])) \
                + k2*((X2[0]-X2[4])**2/(aC**2) + (X2[1]-X2[5])**2/(bC**2) - X2[7]**2)

        x0_2 = np.array([x1_tk, y1_tk, theta1_tk, v1_tk, xC_tk, yC_tk, thetaC_tk, vC_tk])
        lb_2 = np.array([x1_tk - s_x1, y1_tk - s_y1, theta1_tk - s_theta1, v1_tk - s_v1, xC_tk - s_xc, yC_tk - s_yc,
                         thetaC_tk - s_thetaC, vC_tk - s_vc])

        ub_2 = np.array([x1_tk + s_x1, y1_tk + s_y1, theta1_tk + s_theta1, v1_tk + s_v1, xC_tk + s_xc, yC_tk + s_yc,
                         thetaC_tk + s_thetaC, vC_tk + s_vc])

        L_f2_optimization = minimize(
            fun=lambda x: objective01(x),
            x0=x0_2,
            constraints={'type': 'ineq', 'fun': lambda x: ellipse_safety_2(x)},
            bounds=[(lb_2[0], ub_2[0]), (lb_2[1], ub_2[1]), (lb_2[2], ub_2[2]), (lb_2[3], ub_2[3]),
                    (lb_2[4], ub_2[4]),
                    (lb_2[5], ub_2[5]), (lb_2[6], ub_2[6]), (lb_2[7], ub_2[7])]
        )

        L_f2 = L_f2_optimization["fun"]

        if phi_1 >= 0:
            constant = 1
            def objective11(X2):
                return -2*(X2[0]-X2[4])/(aC**2)*(X2[3]*np.sin(X2[2])) + 2*(X2[1]-X2[5])/(bC**2)*(X2[3]*np.cos(X2[2]))

        else:
            constant = -1
            def objective11(X2):
                return 2*(X2[0]-X2[4])/(aC**2)*(X2[3]*np.sin(X2[2])) - 2*(X2[1]-X2[5])/(bC**2)*(X2[3]*np.cos(X2[2]))

        L_g21_optimization = minimize(
            fun=lambda x: objective11(x),
            x0=x0_2,
            constraints={'type': 'ineq', 'fun': lambda x: ellipse_safety_2(x)},
            bounds=[(lb_2[0], ub_2[0]), (lb_2[1], ub_2[1]), (lb_2[2], ub_2[2]), (lb_2[3], ub_2[3]),
                    (lb_2[4], ub_2[4]),
                    (lb_2[5], ub_2[5]), (lb_2[6], ub_2[6]), (lb_2[7], ub_2[7])]
        )
        L_g21 = constant * L_g21_optimization["fun"]


        if phi_C >= 0:
            constant = 1
            def objective21(X2):
                return 2 * (X2[0] - X2[4]) / (aC**2) * (X2[7] * np.sin(X2[6])) - \
                   2 * (X2[1] - X2[5]) / (bC**2) * (X2[7] * np.cos(X2[6]))
        else:
            constant = -1
            def objective21(X2):
                return -2 * (X2[0] - X2[4]) / (aC**2) * (X2[7] * np.sin(X2[6])) + \
                    2 * (X2[1] - X2[5]) / (bC**2) * (X2[7] * np.cos(X2[6]))


        L_g22_optimization = minimize(
            fun=lambda x: objective21(x),
            x0=x0_2,
            constraints={'type': 'ineq', 'fun': lambda x: ellipse_safety_2(x)},
            bounds=[(lb_2[0], ub_2[0]), (lb_2[1], ub_2[1]), (lb_2[2], ub_2[2]), (lb_2[3], ub_2[3]),
                    (lb_2[4], ub_2[4]),
                    (lb_2[5], ub_2[5]), (lb_2[6], ub_2[6]), (lb_2[7], ub_2[7])]
        )
        L_g22 = constant * L_g22_optimization["fun"]

        if u1 >= 0:
            constant = 1
            def objective31(X):
                return -2 * X[3]
        else:
            constant = -1
            def objective31(X):
                return 2 * X[3]

        L_g23_optimization = minimize(
            fun=lambda x: objective31(x),
            x0=x0_2,
            constraints={'type': 'ineq', 'fun': lambda x: ellipse_safety_2(x)},
            bounds=[(lb_2[0], ub_2[0]), (lb_2[1], ub_2[1]), (lb_2[2], ub_2[2]), (lb_2[3], ub_2[3]),
                    (lb_2[4], ub_2[4]),
                    (lb_2[5], ub_2[5]), (lb_2[6], ub_2[6]), (lb_2[7], ub_2[7])]
        )

        L_g23 = constant * L_g23_optimization["fun"]
        LfB2 = L_f2
        LgB2 = np.array([0, -L_g23, -L_g22, -L_g21, 0, 0, 0, 0])
        classK2 = 0

        A = np.append(A, [LgB2], axis=0)
        b = np.append(b, [LfB2 + classK2])


        LfV1 = 0
        LgV1 = np.array([2 * (xC[3] - v_des), 0, 0, 0, -1, 0, 0, 0])
        classV1 = vareps1 * (xC[3] - v_des) ** 2

        A = np.append(A, [LgV1], axis=0)
        b = np.append(b, [-LfV1 -classV1])

        v_des = 30
        LfV2 = 0
        LgV2 = [0, 2 * (x1[3] - v_des), 0, 0, 0, 0, -1, 0]
        classV2 = vareps2 * (x1[3] - v_des) ** 2

        A = np.append(A, [LgV2], axis=0)
        b = np.append(b, [-LfV2 -classV2])


        # u_c, u_1, steer_c, steer_1, v_C_des, Cav_c_lane_change, v_u_des , lyp4

        LfV3 = 2 * (xC[1] - lanewidth-2) * xC[3] * np.sin(xC[2])
        LgV3 = np.array([0, 0, 2 * (xC[1] - lanewidth-2) * xC[3] * np.cos(xC[2]), 0, 0, -1, 0, 0])
        classV3 = 100 * vareps3 * (xC[1] - lanewidth-2) ** 2

        A = np.append(A, [LgV3], axis=0)
        b = np.append(b, [-LfV3 - classV3])


        # u_c, u_1, steer_c, steer_1, v_C_des, Cav_c_lane_change, v_u_des , Cav_1_lane_keep
        LfV6 = 2 * (x1[1] - lanewidth-2) * x1[3] * np.sin(x1[2])
        LgV6 = [0, 0, 0, 2 * (x1[1] - lanewidth-2) * x1[3] * np.cos(x1[2]), 0, 0, 0, -1]
        classV6 = vareps6 * (x1[1] - lanewidth-2) ** 2

        A = np.append(A, [LgV6], axis=0)
        b = np.append(b, [-LfV6 - classV6])


        def objective02(X_1):
            return 2 * (X_1[4] - X_1[0] - X_1[8]) / (aH ** 2) * (
                        X_1[7] * np.cos(X_1[6]) - X_1[3] * np.cos(X_1[2]) - self.h[0] - X_1[10]) + \
            2 * (X_1[5] - X_1[1] - X_1[9]) / (bH ** 2) * (
                        X_1[7] * np.sin(X_1[6]) - X_1[3] * np.sin(X_1[2]) - self.h[1] - X_1[11]) + \
            k1 * ((X_1[4] - X_1[0] - X_1[8]) ** 2 / (aH ** 2) + (X_1[5] - X_1[1] - X_1[9]) ** 2 / (bH ** 2) - X_1[
                7] ** 2)

        x0_1 = [xH_tk, yH_tk, thetaH_tk, vH_tk, xC_tk, yC_tk, thetaC_tk, vC_tk, 0, 0, 0, 0]
        lb_1 = [xH_tk - s_xH, yH_tk - s_yH, thetaH_tk - s_thetaH, vH_tk - s_vH, xC_tk - s_xc, yC_tk - s_yc,
                thetaC_tk - s_thetaC, vC_tk - s_vc, -ex_b, -ey_b, -ex_dot_b, -ey_dot_b]
        ub_1 = [xH_tk + s_xH, yH_tk + s_yH, thetaH_tk + s_thetaH, vH_tk + s_vH, xC_tk + s_xc, yC_tk + s_yc,
                thetaC_tk + s_thetaC, vC_tk + s_vc, ex_b, ey_b, ex_dot_b, ey_dot_b]

        L_fb1_optimization = minimize(
            fun=lambda x: objective02(x),
            x0=x0_1,
            constraints={'type': 'ineq', 'fun': lambda x: ellipse_safety_1(x)},
            bounds=[(lb_1[0], ub_1[0]), (lb_1[1], ub_1[1]), (lb_1[2], ub_1[2]), (lb_1[3], ub_1[3]),
                    (lb_1[4], ub_1[4]),(lb_1[5], ub_1[5]), (lb_1[6], ub_1[6]), (lb_1[7], ub_1[7])
                    , (lb_1[8], ub_1[8]), (lb_1[9], ub_1[9]), (lb_1[10], ub_1[10]), (lb_1[11], ub_1[11])]
        )

        L_fb1 = L_fb1_optimization["fun"]

        if phi_C >= 0:
            constant = 1
            def objeective12(X_1):
                return  2 * (X_1[4] - X_1[0] - X_1[8]) / (aH**2) * (-X_1[7] * np.sin(X_1[6])) +\
                    2 * (X_1[5] - X_1[1] - X_1[9]) / (bH**2) * (X_1[7] * np.cos(X_1[6]))
        else:
            constant = -1
            def objeective12(X_1):
                return  -2 * (X_1[4] - X_1[0] - X_1[8]) / (aH**2) * (-X_1[7] * np.sin(X_1[6])) -\
                    2 * (X_1[5] - X_1[1] - X_1[9]) / (bH**2) * (X_1[7] * np.cos(X_1[6]))


        L_gb11_optimization = minimize(
            fun=lambda x: objeective12(x),
            x0=x0_1,
            constraints={'type': 'ineq', 'fun': lambda x: ellipse_safety_1(x)},
            bounds=[(lb_1[0], ub_1[0]), (lb_1[1], ub_1[1]), (lb_1[2], ub_1[2]), (lb_1[3], ub_1[3]),
                    (lb_1[4], ub_1[4]),(lb_1[5], ub_1[5]), (lb_1[6], ub_1[6]), (lb_1[7], ub_1[7])
                    , (lb_1[8], ub_1[8]), (lb_1[9], ub_1[9]), (lb_1[10], ub_1[10]), (lb_1[11], ub_1[11])]
        )

        L_gb11 = constant * L_gb11_optimization["fun"]


        if uC >= 0 :
            constant = 1
            def objective22(X_1):
                return -2 * X_1[7]
        else:
            constant = -1
            def objective22(X_1):
                return 2 * X_1[7]

        L_gb12_optimization = minimize(
            fun=lambda x: objective22(x),
            x0=x0_1,
            constraints={'type': 'ineq', 'fun': lambda x: ellipse_safety_1(x)},
            bounds=[(lb_1[0], ub_1[0]), (lb_1[1], ub_1[1]), (lb_1[2], ub_1[2]), (lb_1[3], ub_1[3]),
                    (lb_1[4], ub_1[4]), (lb_1[5], ub_1[5]), (lb_1[6], ub_1[6]), (lb_1[7], ub_1[7])
                , (lb_1[8], ub_1[8]), (lb_1[9], ub_1[9]), (lb_1[10], ub_1[10]), (lb_1[11], ub_1[11])]
        )

        L_gb12 = constant * L_gb12_optimization["fun"]

        LfB1 = L_fb1
        LgB1 = np.array([-L_gb12, 0, -L_gb11, 0, 0, 0, 0, 0])
        classK1 = 0

        A = np.append(A, [LgB1], axis=0)
        b = np.append(b, [LfB1 + classK1])


        # safety HDV and CAV 1

        def objective03(X3):
            return  2 * (X3[4] - X3[0] - X3[8]) / (aH**2) * (X3[7] * np.cos(X3[6]) - X3[3] * np.cos(X3[2]) - self.h[0] - X3[10])\
                    +2 * (X3[5] - X3[1] - X3[9]) / (bH**2) * (X3[7] * np.sin(X3[6]) - X3[3] * np.sin(X3[2]) - self.h[1] - X3[11])\
                    + k3 * ((X3[4] - X3[0] - X3[8])**2 / (aH**2) + (X3[5] - X3[1] - X3[9])**2 / (bH**2) - X3[7]**2)\


        x0_3 = [xH_tk, yH_tk, thetaH_tk, vH_tk, x1_tk, y1_tk, theta1_tk, v1_tk, 0, 0, 0, 0]
        lb_3 = [xH_tk - s_xH, yH_tk - s_yH, thetaH_tk - s_thetaH, vH_tk - s_vH, x1_tk - s_x1, y1_tk - s_y1,
                theta1_tk - s_theta1, v1_tk - s_v1, -ex_b, -ey_b, -ex_dot_b, -ey_dot_b]
        ub_3 = [xH_tk + s_xH, yH_tk + s_yH, thetaH_tk + s_thetaH, vH_tk + s_vH, x1_tk + s_x1, y1_tk + s_y1,
                theta1_tk + s_theta1, v1_tk + s_v1, ex_b, ey_b, ex_dot_b, ey_dot_b]

        L_fb3_optimization = minimize(
            fun=lambda x: objective03(x),
            x0=x0_3,
            constraints={'type': 'ineq', 'fun': lambda x: ellipse_safety_3(x)},
            bounds=[(lb_3[i], ub_3[i]) for i in range(len(lb_3))]
        )

        L_fb3 = L_fb3_optimization["fun"]


        if phi_1 >= 0:
            constant = 1
            def objective31(X3):
                return -2 * (X3[4] - X3[0] - X3[8]) / (aH**2) * (X3[7] * np.sin(X3[6])) + \
                        2 * (X3[5] - X3[1] - X3[9]) / (bH**2) * (X3[7] * np.cos(X3[6]))

        else:
            constant = -1
            def objective31(X3):
                return  2 * (X3[4] - X3[0] - X3[8]) / (aH**2) * (X3[7] * np.sin(X3[6])) - \
                        2 * (X3[5] - X3[1] - X3[9]) / (bH**2) * (X3[7] * np.cos(X3[6]))


        L_gb31_optimization = minimize(
            fun=lambda x: objective31(x),
            x0=x0_3,
            constraints={'type': 'ineq', 'fun': lambda x: -1},
            bounds=[(lb_3[i], ub_3[i]) for i in range(len(lb_3))]
        )

        L_gb31 = constant * L_gb31_optimization["fun"]


        if u1 >= 0:
            constant = 1
            def objective32(X3):
                return -2 * X3[7]
        else:
            constant = -1
            def objective32(X3):
                return 2 * X3[7]


        L_gb32_optimization = minimize(
            fun=lambda x: objective32(x),
            x0=x0_3,
            constraints={'type': 'ineq', 'fun': lambda x: ellipse_safety_3(x)},
            bounds=[(lb_3[i], ub_3[i]) for i in range(len(lb_3))]
        )

        L_gb32 = constant * L_gb32_optimization["fun"]


        LfB3 = L_fb3
        LgB3 = np.array([0, -L_gb32, 0, -L_gb31, 0, 0, 0, 0])
        classK3= 0

        A = np.append(A, [LgB3], axis=0)
        b = np.append(b, [LfB3 + classK3])



        H = np.eye(8)
        H[0,0] = 0.01
        H[1,1] = 0.01
        f = np.zeros((8, 1))


        P = matrix(H)
        q = matrix(f)
        G = matrix(A)  # cvxopt uses <= inequality, so multiply by -1
        h = matrix(b)

        # try :
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
        #self.CBF_with_CAV1.append(0 * u[0] +L_g23 * u[1] +L_g22 * u[2] +L_g21 * u[3] + L_f2)
        # self.CBF_with_CAV1.append(0 * u[0] + L_gb32 * u[1] + 0 * u[2] + L_gb31 * u[3] + LfB3)
        # self.CBF_with_CAV1.append(L_fbU-L_gbU3*u[0] -L_gbU2*u[2])
        self.CBF_with_CAV1.append(L_fb1+L_gb12*u[0]+L_gb11*u[2])
        #print(u[0],xC[3],u[1],x1[3])
        # print(x1[3] * np.cos(x1[2])*u[3]+ LfB7 + classK7, -L_gb32*u[1]-L_gb31*u[3]+ LfB3)
        # print(L_fbU + L_gbU3 * u[0] + L_gbU2 * u[1], u[0])
        # except:
        #     # Handle the case when no optimal solution is found
        #     u = matrix([self.umin/2, 0.0])  # [-cd * m * g, 0]
        if u[1] > 0.01 or u[1] < -0.01:
            stop = 1

        self.h[0] += ex_dot
        self.h[1] += ey_dot
        self.h[2] += etheta_dot
        self.h[3] += ev_dot
        # print(u[4:])
        return u[0:4]

class Truck(Vehicle):

    def __init__(self, states, model=None):
        super().__init__(states)
        self.length = 7
        self.width = 2
        self.color = 'gray'
        self.corner = np.array([[-self.length/2, -self.width/2], [-self.length/2, +self.width/2],
                                [+self.length/2, +self.width/2], [+self.length/2, -self.width/2], [-self.length/2, -self.width/2]])

    def dynamics(self, t, x):
        dx = [0] * 6
        dx[0] = x[3] * np.cos(x[2])
        dx[1] = x[3] * np.sin(x[2])
        dx[2] = (x[3] * x[5]) / (0.39 + 0.39)
        dx[3] = x[4]
        dx[4] = 0
        dx[5] = 0

        return dx


#
# def callibrate(vehicle,ut,ub,phi):
#     throttle = (1 - ut)/2 * vehicle.umax
#     brake = (ub - 1)/2 * vehicle.umin
#     acc = throttle - brake
#     steer = phi * abs(vehicle.steermax)
#     return acc,steer

# map = Map(render_mode=True)
# hdv = HDV([0, 0, 0, 0])
# # truck = Truck([0, 0, 0, 0])
# # cav = CAV([0, 0, 0, 0])
# # datahdv = []
# # datatruck = []
# # datacav = []
#
# corner = np.array([[-2.5, -1.0], [-2.5, +1.0], [+2.5, +1.0], [+2.5, -1.0], [-2.5, -1.0]])
# for i in range(0, 1000):
#     ut = -i/1000
#     ub = 1
#     phi = 0.001
#     hdv.acc, hdv.phi = callibrate(hdv, ut, ub, phi)
#     print(hdv.acc, hdv.phi)
#     # hdv.acc, hdv.phi = [1, 0.01]
#     hdv._move([0, 1], 1)
#     xH = hdv.states
#     map.render(hdv)
#
#
# plt.show(block=True)


