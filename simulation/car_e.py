import pygame
import sys
import time
import math
from utils import *


class Car_Props:
    def __init__(self, max_vel, rotation_vel, poss):
        self.img = self.img
        self.max_vel = max_vel
        self.vel = 0
        self.rotation_vel = rotation_vel
        self.x, self.y = poss
        self.acceleration = 0.05

    def draw(self, gameDisplay):
        blit_rotate_center(gameDisplay, self.img, (self.x, self.y), self.angle)

    def move(self, vel):
        radians = math.radians(self.angle) 
        vertical = math.cos(radians) * vel
        horizontal = math.sin(radians) * vel

        self.y -= vertical   
        self.x -= horizontal 