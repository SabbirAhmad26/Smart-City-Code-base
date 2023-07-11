import pygame
import sys
import time
import math
import random
from utils import *
from car import *
from spline import *

pygame.font.init()

street = pygame.image.load("imgs/street.png")
width, height = street.get_width(), street.get_height()
gameDisplay = pygame.display.set_mode((width, height))

pygame.display.set_caption("Simulation")
font = pygame.font.SysFont("comicsans", 44)

car_up = scale_image(pygame.image.load("imgs/car.png"), 0.45)

def collideLineLine(P0, P1, Q0, Q1):
    d = (P1[0]-P0[0]) * (Q1[1]-Q0[1]) + (P1[1]-P0[1]) * (Q0[0]-Q1[0]) 
    if d == 0:
        return None
    t = ((Q0[0]-P0[0]) * (Q1[1]-Q0[1]) + (Q0[1]-P0[1]) * (Q0[0]-Q1[0])) / d
    u = ((Q0[0]-P0[0]) * (P1[1]-P0[1]) + (Q0[1]-P0[1]) * (P0[0]-P1[0])) / d
    return 0 <= t <= 1 and 0 <= u <= 1

def colideRectLine(rect, p1, p2):
    return (collideLineLine(p1, p2, rect.topleft, rect.bottomleft) or
            collideLineLine(p1, p2, rect.bottomleft, rect.bottomright) or
            collideLineLine(p1, p2, rect.bottomright, rect.topright) or
            collideLineLine(p1, p2, rect.topright, rect.topleft))

def collideRectPolygon(rect, polygon):
    for i in range(len(polygon)-1):
        if colideRectLine(rect, polygon[i], polygon[i+1]):
            return True
    return False

class GameInfo:
    def __init__(self):
        self.started = False
        self.start_time = 0

    def reset(self):
        self.started = False
        self.start_time = 0

    def game_finished(self):
        return False

    def start_run(self):
        self.started = True
        self.start_time = time.time()

def draw(display, images, car, game_info):
    for img, pos in images:
        display.blit(img, pos)

    car.draw(display)
    pygame.display.update()
    
    
class Car(AbstractCar):
    img = car_up
    
    def __init__(self, max_vel, rotation_vel, angle, poss, path=[]):
        super().__init__(max_vel, rotation_vel, poss)
        self.current_point = 0
        self.vel = max_vel
        self.angle = angle
        self.rect = pygame.Rect(poss[0], poss[1], 17, 34)
        self.time = 0
        self.path = path

    def draw_points(self, win):
        for point in self.path:
            pygame.draw.circle(gameDisplay, (255, 0, 0), point, 5)
            
    def draw(self, gameDisplay):
        super().draw(gameDisplay)
            
    def calculate_angle(self, vel):
        target_x, target_y = self.path[self.current_point]
        x_diff = target_x - self.x
        y_diff = target_y - self.y

        if y_diff == 0:
            final_angle = math.pi / 2
        else:
            final_angle = math.atan(x_diff / y_diff)

        if target_y > self.y:
            final_angle += math.pi

        angle_diff = self.angle - math.degrees(final_angle)
        if angle_diff >= 180:
            angle_diff -= 360

        if angle_diff > 0:
            self.angle -= min(self.rotation_vel, abs(angle_diff))
        else:
            self.angle += min(self.rotation_vel, abs(angle_diff))
            
        distance = (math.sqrt((x_diff/40)**2 + (y_diff/40)**2))* 13.5    #inches
        return distance, self.angle
    
    def update_path_point(self):
        target = self.path[self.current_point]
        rect = pygame.Rect(
            self.x, self.y, self.img.get_width(), self.img.get_height())
        if rect.collidepoint(*target):
            self.current_point += 1

    def move(self, vel):
        if self.current_point >= len(self.path):
            return 0, 0
        
        distance, angle = self.calculate_angle(vel)
        self.update_path_point()
        self.time += delta_time
        super().move(vel)
        return distance, angle


fps = 50
delta_time = 1/fps
clock = pygame.time.Clock()
run = True

up_angle, down_angle, left_angle, right_angle = 0, 180, 90, -90
angle = [up_angle, down_angle, left_angle, right_angle]

poss_start = [(283,411), (159,8), (423, 150), (17, 270)]

start_pos = poss_start[0]
start_angle = angle[0]
images = [(street, (0,0))]
game_info = GameInfo()

path = [(286, 330), (293, 307), (304, 294), (329, 287), (441, 275)]
pts = [(286, 330), (293, 307), (304, 294), (329, 287), (441, 275)]
# formula = poly_calc(pts)

# for i in range(5):
#     x = 0
#     y = 0
#     for j in range(len(formula[0])):
#         if j != len(formula[0])-1:
#             x += int(formula[0][j] * ((i/5)**(len(formula[0])-j)))
#             y += int(formula[1][j] * ((i/5)**(len(formula[1])-j)))
#         else:
#             x += int(formula[0][j])
#             y += int(formula[1][j])
#     path.append((x,y)) 

car = Car(1, 10, start_angle, start_pos, path)

pygame.time.set_timer(pygame.USEREVENT, 5000)

w, h = pygame.display.get_surface().get_size()

border = [(108, 8), (347, 8), (347, 128), (446, 128), (448, 325), (346, 327), (347, 444), (108, 445), (106, 327), (7, 326), (8, 129), (107, 128), (108,8)]
line1 = [(228, 307), (228, 446)]
line2 = [(227, 145), (227, 8)]
line3 = [(147, 227), (8, 227)]
line4 = [(308, 228), (446, 227)]

turning_pts = [(287, 326), (167, 127), (329, 168), (129, 288)]

distance = 0

while run:
    clock.tick(fps)
    
    draw(gameDisplay, images, car, game_info)
    
    while not game_info.started:
        blit_text_center(gameDisplay, font, f"Press any key to start")
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                break
            if event.type == pygame.KEYDOWN:
                game_info.start_run()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
            break
        
    d, angle = car.move(1)
    # print(angle)
    
pygame.quit()