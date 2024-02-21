from scipy.integrate import odeint
import numpy as np
from scipy.integrate import odeint, solve_ivp, ode
import matplotlib.pyplot as plt
from map import Map
from vehicle import CAV, HDV, Truck
import pygame
import logitechG29_wheel
import socket
import time



def callibrate(vehicle,ut,ub,phi):
    throttle = (1 - ut)/2 * vehicle.umax
    brake = (ub - 1)/2 * vehicle.umin
    acc = throttle - brake
    steer = phi * abs(vehicle.steermax)
    return acc, steer

def main():
    pygame.init()
    clock = pygame.time.Clock()

    # make a controller
    controller = logitechG29_wheel.Controller(0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


    map = Map(render_mode=True)
    hdv = HDV([20, 6, 0, 25])
    truck = Truck([60, 0, 0, 20])
    cav_c = CAV([40, 0, 0, 25])
    cav_1 = CAV([60, 6, 0, 29])
    pre_hdv_states = [np.nan] * 4
    # cav = CAV([0, 0, 0, 0])
    # datahdv = []
    # datatruck = []
    # datacav = []
    dt = 0.05

    for i in range(0, 1000):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        jsButtons = controller.get_buttons()
        jsInputs = controller.get_axis()

        steerPos = controller.get_steer()
        throtPos = controller.get_throttle()
        breakPos = controller.get_break()
        print(throtPos,breakPos,hdv.v)
        clutchPos = controller.get_clutch()


        # ut = -i/1000
        # ub = 1
        # phi = 0.001
        hdv.acc_manual, hdv.steer_manual = callibrate(hdv, throtPos, breakPos, steerPos)
        # ut = -i/1000
        # ub = 1
        # phi = 0
        # hdv.acc, hdv.phi = callibrate(hdv, ut, ub, phi)
        truck.acc, truck.steer = 0.0, 0.0
        hdv.acc, hdv.steer = hdv.control()
        yh = np.copy(hdv.states)
        cav_c.acc, cav_1.acc, cav_c.steer, cav_1.steer = cav_c.control(truck.states, cav_1.states, cav_1.acc, cav_1.steer,
                                                                       yh, hdv.states, hdv.acc, hdv.phi, pre_hdv_states)

        pre_hdv_states = hdv.states
        hdv._move([0, dt], dt, True)
        truck._move([0, dt], dt, False)
        cav_c._move([0, dt], dt, False)
        cav_1._move([0, dt], dt, False)
        # print(hdv.acc)
        map.render([cav_1,cav_c, hdv, truck])


    # plt.show(block=True)


if __name__ == '__main__':
    main()