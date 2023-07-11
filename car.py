import pygame
import sys
import time
import math
from utils import *


class AbstractCar:
    def __init__(self, max_vel, rotation_vel, poss):
        self.img = self.img
        self.max_vel = max_vel
        self.vel = 0
        self.rotation_vel = rotation_vel
        self.x, self.y = poss
        self.acceleration = 0.01

    def rotate(self, left=False, right=False):
        if left:
            self.angle += self.rotation_vel
        elif right:
            self.angle -= self.rotation_vel

    def draw(self, gameDisplay):
        blit_rotate_center(gameDisplay, self.img, (self.x, self.y), self.angle)

    def move_forward(self):
        self.vel = min(self.vel + self.acceleration, self.max_vel)
        self.move()

    def move_backward(self):
        self.vel = max(self.vel - self.acceleration, -self.max_vel/2)
        self.move()

    def move(self, vel):
        radians = math.radians(self.angle)
        vertical = math.cos(radians) * vel
        horizontal = math.sin(radians) * vel

        self.y -= vertical
        self.x -= horizontal