import numpy as np
import cvxopt.solvers
from cvxopt import matrix


def dynamics(xH,uH,phi_H):

    umin = -6
    umax = 3

    uH_ref = uH
    phi_H_ref = phi_H

    H_hdv = np.eye(4)
    F_hdv = np.array([-2*uH_ref, -2*phi_H_ref, 0, 0])
    A_hdv = np.array([[0, -xH[3]*np.cos(xH[2]), 0, 0],
                      [0, xH[3]*np.cos(xH[2]), 0, 0]])

    b_hdv = np.array([xH[3]*np.sin(xH[2]) + k8*(xH[1] + lanewidth/2),
                      -xH[3]*np.sin(xH[2]) + k9*(3*lanewidth/2 - xH[1])])

    lb_hdv = [-np.inf, -np.inf, 0, 0]
    ub_hdv = [np.inf, np.inf, np.inf, np.inf]

    P = matrix(H_hdv)
    q = matrix(F_hdv)
    G = matrix(A_hdv)  # cvxopt uses <= inequality, so multiply by -1
    H = matrix(b_hdv)

    try:
        options = {'show_progress': False}
        Solution = cvxopt.solvers.qp(P, q, G, H, options=options)
        if Solution["status"] == 'optimal':
            u = Solution['x']
        else:
            u = Solution['x']
            if u[0] > umax:
                u = [umax, 0.0, 0.0, 0.0]
            else:
                u = [umin, 0.0, 0.0, 0.0]
    except:
        u = [umin, 0.0, 0.0, 0.0]

    uH, phi_H = u[0:2]

    h1 = h1 + ex_dot
    h2 = h2 + ey_dot
    h3 = h3 + etheta_dot
    h4 = h4 + ev_dot

    return