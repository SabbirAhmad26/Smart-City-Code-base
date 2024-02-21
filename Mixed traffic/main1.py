import numpy as np
import matplotlib.pyplot as plt
import matlab.engine
eng = matlab.engine.start_matlab()
import matplotlib.animation as animation


import sys
import pygame
import numpy as np
sys.path.append('C:/Users/anlianni/AppData/Local/Programs/Python/Python310/lib/site-packages/logidrivepy')
from logidrivepy import LogitechController

import pygetwindow as gw
import logitech_steering_wheel as lsw
import time

controller = LogitechController()
clock = pygame.time.Clock()

print(f"steering_initialize: {controller.steering_initialize()}")
print(f"logi_update: {controller.logi_update()}")
print(f"is_connected: {controller.is_connected(0)}")

# window_handle = gw.getActiveWindow()._hWnd
# initialized = lsw.initialize_with_window(ignore_x_input_controllers=True, hwnd=window_handle)

# print("SDK version is: " + str(lsw.get_sdk_version()))

# connected = lsw.is_connected(0)

# lsw.update()


# operated = lsw.set_operating_range(-450, 450)

# import pdb; pdb.set_trace()



# initial settings
te = 600
Lw = 4
lanewidth = 4
N = 1  # repeat times

def map():
    fig, ax = plt.subplots(figsize = (40,8))
    ax.axis('equal')
    
    lineU, = ax.plot([],[], color = 'y', linewidth = 3.0)
    lineC, = ax.plot([],[], color = 'g', linewidth = 3.0)
    line1, = ax.plot([],[], color = 'g', linewidth = 3.0)
    lineH, = ax.plot([],[], color = 'r', linewidth = 3.0)
    return lineU, fig, ax, lineH, line1, lineC


lineU, fig, ax, lineH, line1, lineC = map()



# weights of time and energy
alpha_t = 0.5
alpha_u = 0.5
alpha_phi = 0.3
v_des = 30
v_min, v_max = 15, 35
u_min, u_max = -7, 3.3

# safety ellipse
aH, bH = 0.6, 0.1
aC, bC = 0.6, 0.15
a1, b1 = aC, bC

event = 0

# global U
t_next = 0
t0 = 0
dt = 0.05
h1, h2, h3, h4 = 0, 0, 0, 0
uC, phi_C, u1, phi_1, uH, phi_H = 0, 0, 0, 0, 0, 0

# initial positions
xC = np.array([20, 0, 0, 25])  # [x, y, steering angle, speed]
xH = np.array([10, 4, 0, 28])
x1 = np.array([50, 4, 0, 29])
v_H0 = xH[3]
xU = np.array([xC[0] + 40, 0, 0, 20])  # truck
vU = xU[3]

# record
xC_hist = np.array([[0, *xC]])
x1_hist = np.array([[0, *x1]])
xH_hist_nomi = np.array([[0, *xH]])
xH_hist_real = np.array([[0, *xH]])
xH_hist = np.array([[0, *xH]])
xU_hist = np.array([[0, *xU]])

bch_hist = []
b1c_hist = []
b1h_hist = []
bCU_hist = []

corner = np.array([[-2.5, -1.0], [-2.5, +1.0], [+2.5, +1.0], [+2.5, -1.0], [-2.5, -1.0]])

# events and error bounds
s_xc, s_yc, s_vc, s_thetaC = 0.01, 0.005, 1, 0.01
s_xH, s_yH, s_vH, s_thetaH = 0.01, 0.005, 1, 0.01
s_x1, s_y1, s_v1, s_theta1 = 0.01, 0.005, 1, 0.01
s_xU, s_yU, s_vU, s_thetaU = 0.01, 0.005, 1, 0.01
ex_b, ey_b, ev_b = 0.2, 0.1, 1
ex_dot_b, ey_dot_b, ev_dot_b = 0.5, 0.2, 1
# import pdb; pdb.set_trace()
xU = matlab.double(xU.tolist())[0]
xC = matlab.double(xC.tolist())[0]
xH = matlab.double(xH.tolist())[0]
x1 = matlab.double(x1.tolist())[0]
uC = matlab.double(uC)
phi_C = matlab.double(phi_C)
u1 = matlab.double(u1)
phi_1 = matlab.double(phi_1)
uH = matlab.double(uH)
phi_H = matlab.double(phi_H)
h1 = matlab.double(h1)
h2 = matlab.double(h2)
h3 = matlab.double(h3)
h4 = matlab.double(h4)


# main loop
for i in range(te + 1):
# def update(i):
    # global xC, xH, x1, v_H0, xU, vU, xC_hist, xH_hist, xU_hist, x1_hist, uC, phi_C, u1, phi_1, uH, phi_H, h1, h2, h3, h4, t_next, xH_tk, vH_tk, yH_tk, xC_tk, vC_tk, yC_tk, x1_tk, y1_tk, v1_tk, xU_tk, yU_tk, vU_tk
    # disturbances
    eps1 = 1.4 * (0.5 - np.random.rand())  # [-0.2, 0.2]
    eps2 = 1 * (0.5 - np.random.rand())  # [-0.02, 0.02]
    eps3 = 1 * (0.5 - np.random.rand())  # [-0.01, 0.01]
    eps4 = 1.4 * (0.5 - np.random.rand())  # [-0.2, 0.2]
    sigma1 = 1 + 0.2 * (0.5 - np.random.rand())  # [0.9, 1.1]
    sigma2 = 1 + 0.2 * (0.5 - np.random.rand())  # [0.9, 1.1]

    eps1 = matlab.double(eps1)
    eps2 = matlab.double(eps2)
    eps3 = matlab.double(eps3)
    eps4 = matlab.double(eps4)
    sigma1 = matlab.double(sigma1)
    sigma2 = matlab.double(sigma2)

    # import pdb; pdb.set_trace() #data collection
    bch_hist.append([dt * i, (xC[0] - xH[0]) ** 2 / (aH ** 2) + (xC[1] - xH[1]) ** 2 / (bH ** 2) - xC[3] ** 2])
    b1c_hist.append([dt * i, (x1[0] - xC[0]) ** 2 / (aC ** 2) + (x1[1] - xC[1]) ** 2 / (bC ** 2) - xC[3] ** 2])
    b1h_hist.append([dt * i, (x1[0] - xH[0]) ** 2 / (aH ** 2) + (x1[1] - xH[1]) ** 2 / (bH ** 2) - x1[3] ** 2])
    bCU_hist.append([dt * i, (xU[0] - xC[0]) ** 2 / (aC ** 2) + (xU[1] - xC[1]) ** 2 / (bC ** 2) - xC[3] ** 2])

    # min b value
    # bch = min(bch_hist[-1][1])
    # b1c = min(b1c_hist[-1][1])
    # b1h = min(b1h_hist[-1][1])
    # bCU = min(bCU_hist[-1][1])

    if abs(dt * i - t_next) < 0.0001:
        event += 1
       # import pdb; pdb.set_trace()
        
        controller.logi_update()
        state = controller.get_state_engines(0)
        # import pdb; pdb.set_trace()
        # state.contents.lX  (turn the steering wheel left or right)
        # lY (tap the accelerator)
        # lRz (break)
        time.sleep(0.1)
        # print(state.contents.lX, state.contents.lY, state.contents.lRz)
        phi_H = state.contents.lX * np.pi / 5
        if state.contents.lY == 32767 and state.contents.lRz == 32767:
            uH = 0
        else:
            uH = (-7 - state.contents.lRz * (-7) / 32767) + (3.3 - state.contents.lY * 3.3 /32767)

        print(uH, phi_H)
        
        
        yH = np.copy(xH)  # x: real dynamics, y: nominal dynamics

        rt = eng.event_solve_qp(
            xU,
            xC,
            xH,
            x1,
            uC,
            phi_C,
            u1,
            phi_1,
            uH,
            phi_H,
            h1,
            h2,
            h3,
            h4,
            eps1,
            eps2,
            eps3,
            eps4,
            sigma1,
            sigma2,
            yH,
        )
        # import pdb; pdb.set_trace()
        xU = rt[0][0:4]
        xC = rt[0][4:8]
        xH = rt[0][8:12]
        x1 = rt[0][12:16]
        uC = rt[0][16]
        uH = rt[0][17]
        u1 = rt[0][18]
        phi_1 = rt[0][19]
        phi_H = rt[0][20]
        phi_C = rt[0][21]
        h1 = rt[0][22]
        h2 = rt[0][23]
        h3 = rt[0][24]
        h4 = rt[0][25]
        yH = rt[0][26:30]
        xH_tk = rt[0][30]
        yH_tk = rt[0][31]
        vH_tk = rt[0][32]
        xC_tk = rt[0][33]
        yC_tk = rt[0][34]
        vC_tk = rt[0][35]
        x1_tk = rt[0][36]
        y1_tk = rt[0][37]
        v1_tk = rt[0][38]
        xU_tk = rt[0][39]
        yU_tk = rt[0][40]
        vU_tk = rt[0][41]
    
    rot = np.array([[np.cos(xU[2]), -np.sin(xU[2])],[np.sin(xU[2]), np.cos(xU[2])]])
    rot_corner = rot.dot(corner.transpose()).transpose()
    lineU.set_data(xU[0] + rot_corner[:,0], xU[1] + rot_corner[:,1])

    rotC = np.array([[np.cos(xC[2]), -np.sin(xC[2])],[np.sin(xC[2]), np.cos(xC[2])]])
    rot_corner_C = rotC.dot(corner.transpose()).transpose()
    lineC.set_data(xC[0] + rot_corner_C[:,0], xC[1] + rot_corner_C[:,1])

    rot1 = np.array([[np.cos(x1[2]), -np.sin(x1[2])],[np.sin(x1[2]), np.cos(x1[2])]])
    rot_corner_1 = rot1.dot(corner.transpose()).transpose()
    line1.set_data(x1[0] + rot_corner_1[:,0], x1[1] + rot_corner_1[:,1])

    rotH = np.array([[np.cos(xH[2]), -np.sin(xH[2])],[np.sin(xH[2]), np.cos(xH[2])]])
    rot_corner_H = rotH.dot(corner.transpose()).transpose()
    lineH.set_data(xH[0] + rot_corner_H[:,0], xH[1] + rot_corner_H[:,1])

    xpoints = np.array([-10,2000])
    ypoints1 = np.array([-2,-2])
    ypoints2 = np.array([2,2])
    ypoints3 = np.array([6,6])

    plt.plot(xpoints,ypoints1)
    plt.plot(xpoints,ypoints2)
    plt.plot(xpoints,ypoints3)
    plt.show


    # import pdb; pdb.set_trace()
    xU_hist = np.vstack([xU_hist, [dt * i, *xU]])
    xC_hist = np.vstack([xC_hist, [dt * i, *xC]])
    x1_hist = np.vstack([x1_hist, [dt * i, *x1]])
    xH_hist = np.vstack([xH_hist, [dt * i, *xH]])
    xH_hist_nomi = np.vstack([xH_hist_nomi, [dt * i, *yH]])

    # check the terimal y state of CAV C
    if lanewidth - 0.3 <= xC[1] <= lanewidth + 0.3:
        tf = dt * i
        break

    ex = xH[0] - yH[0]
    ey = xH[1] - yH[1]
    ev = xH[3] - yH[3]
    eps1 = matlab.double(eps1)
    eps2 = matlab.double(eps2)
    eps3 = matlab.double(eps3)
    eps4 = matlab.double(eps4)
    # import pdb; pdb.set_trace()
    ex_dot = (xH[3] * np.cos(xH[2]) * sigma1 - xH[3] * np.sin(xH[2]) * phi_H + eps1) - (yH[3] * np.cos(yH[2]) + h1)
    # ex_dot = (xH[3] - xh[3]_pre)/0.05 - (yH[3] * np.cos(yH[2]) + h1)
    ey_dot = (xH[3] * np.sin(xH[2]) * sigma2 + xH[3] * np.cos(xH[2]) * phi_H + eps2) - (yH[3] * np.sin(yH[2]) + h2)
    ev_dot = (uH + eps4[0][0]) - h4
    if (
        yH[0] < xH_tk - s_xH
        or yH[0] > xH_tk + s_xH
        or yH[1] < yH_tk - s_yH
        or yH[1] > yH_tk + s_yH
        or yH[3] < vH_tk - s_vH
        or yH[3] > vH_tk + s_vH
        or xC[0] < xC_tk - s_xc
        or xC[0] > xC_tk + s_xc
        or xC[1] < yC_tk - s_yc
        or xC[1] > yC_tk + s_yc
        or xC[3] < vC_tk - s_vc
        or xC[3] > vC_tk + s_vc
        or x1[0] < x1_tk + s_x1
        or x1[0] > x1_tk + s_x1
        or x1[1] < y1_tk - s_y1
        or x1[1] > y1_tk + s_y1
        or x1[3] < v1_tk - s_v1
        or x1[3] > v1_tk + s_v1
    ):
        t_next = dt + t0
    else:
        t_next = dt * te

    t0 = t_next
    ax.axis([-80 + xU[0], 80 + xU[0], -16, 16])
    plt.pause(0.02)
    fig.canvas.draw()
    
plt.show(block = True)


# ani = animation.FuncAnimation(fig, update, te+1, fargs=[],
#                                 interval=25, blit=False, repeat = False)  # interval/ms, blit = False/without return
# ani.save('./video.mp4')  
# plt.show()