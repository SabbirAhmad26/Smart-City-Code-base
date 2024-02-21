from vehicle import HDV
import sys
import pygame
import numpy as np
# sys.path.append('C:/Users/anlianni/AppData/Local/Programs/Python/Python310/lib/site-packages/logidrivepy')
from logidrivepy import LogitechController
import time


controller = LogitechController()
clock = pygame.time.Clock()

print(f"steering_initialize: {controller.steering_initialize()}")
print(f"logi_update: {controller.logi_update()}")
print(f"is_connected: {controller.is_connected(0)}")

# # import pdb; pdb.set_trace()
# # state.contents.lX  (turn the steering wheel left or right)
# # lY (tap the accelerator)
# # lRz (break)
# time.sleep(0.1)
# # print(state.contents.lX, state.contents.lY, state.contents.lRz)
controller.logi_update()
state = controller.get_state_engines(0)
hdv = HDV([0, 0, 0, 0])
hdv.phi = state.contents.lX * 1.25 * np.pi / 32767
if state.contents.lY == 32767 and state.contents.lRz == 32767:
    hdv.acc = 0
else:
    hdv.acc = (-7 - state.contents.lRz * (-7) / 32767) + (3.3 - state.contents.lY * 3.3 /32767)