import pygame
import sys
import time
import math
import random
from utils import *
from car_e import *
from circle_e import *

pygame.font.init()

street = pygame.image.load("imgs/extension.png")
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

class GameStart:
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

# def coord_trans(coord, trans):    #inch to px 
#     x = int((coord[0]-54.2)*40/13.5)+8
#     y = h - 8 + int((coord[1]+6.7)*40/13.5)
#     return (x,y)

def coord_trans(coord, trans):    #inch to px 
    x = int((coord[0]-54.2)*40/13.5)+8
    y = h - 8 + int((coord[1]+6.7)*40/13.5)
    return (y,x)

def coord_trans_list(coord):
    for i in len(coord):
        i = coord_trans(coord[i])
    return coord

def angle_trans(angle):
    for i in angle:
        i -= 90
    return angle
    
class Car(Car_Props):
    img = car_up
    
    def __init__(self, max_vel, rotation_vel, angle, poss, path=[]):
        super().__init__(max_vel, rotation_vel, poss)
        self.current_point = 0
        self.vel = max_vel
        self.angle = angle
        self.rect = pygame.Rect(poss[0], poss[1], 17, 34)
        self.time = 0
        self.path = path
        self.delta_time = 0.25

    def draw_points(self, win):
        for point in self.path:
            pygame.draw.circle(gameDisplay, (255, 0, 0), point, 5)
            
    def draw(self, gameDisplay):
        super().draw(gameDisplay)
        self.draw_points(gameDisplay)
            
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
            
        distance = (math.sqrt((x_diff)**2 + (y_diff)**2))/40*13.5    #inches
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
    
    def timer(self):
        return self.time
    
    def project(self):
        if start_angle == 0:
            if self.y > turning_pts[0][1] or self.angle == 0:
                return (self.x - poss_start[0][0])/40*13.5
            else:
                if (self.angle+90 < 7) and (self.angle+90 > -7):
                    return (self.y - poss_start[3][1])/40*13.5
                else:
                    if self.angle < 0:
                        return (math.sqrt((self.x-right_circle[0][1][0])**2 + (self.y-right_circle[0][1][1])**2) - right_circle[0][0])/40*13.5
                    else:
                        return (math.sqrt((self.x-left_circle[0][1][0])**2 + (self.y-left_circle[0][1][1])**2) - left_circle[0][0])/40*13.5
                    
        elif start_angle == 180:
            if self.y < turning_pts[1][1] or self.angle == 180:
                return (self.x - start_pos[1])/40*13.5
            else:
                if (self.angle - 90 < 7) and (self.angle-90 > -7):
                    return (self.y - poss_start[2][1])/40*13.5
                else:
                    if self.angle < 180:
                        return (math.sqrt((self.x-right_circle[1][1][0])**2 + (self.y-right_circle[1][1][1])**2) - right_circle[1][0])/40*13.5
                    else:
                        return (math.sqrt((self.x-left_circle[1][1][0])**2 + (self.y-left_circle[1][1][1])**2) - left_circle[1][0])/40*13.5
                
        elif start_angle == 90:
            if self.x > turning_pts[1][0] or self.angle == 90:
                return (self.y - start_pos[2])/40*13.5
            else:
                if (self.angle < 7) and (self.angle > -7):
                    return (self.x - poss_start[0][0])/40*13.5
                else:
                    if self.angle < 90:
                        return (math.sqrt((self.x-right_circle[2][1][0])**2 + (self.y-right_circle[2][1][1])**2) - right_circle[2][0])/40*13.5
                    else:
                        return (math.sqrt((self.x-left_circle[2][1][0])**2 + (self.y-left_circle[2][1][1])**2) - left_circle[2][0])/40*13.5
                
        else:
            if self.x < turning_pts[2][0] or (self.angle == 270):
                return (self.y - start_pos[3])/40*13.5
            else:
                if (self.angle-180 < 7) or (self.angle-180 > -7):
                    return (self.x - poss_start[1][0])/40*13.5
                else:
                    if self.angle < 270:
                        return (math.sqrt((self.x-right_circle[3][1][0])**2 + (self.y-right_circle[3][1][1])**2) - right_circle[3][0])/40*13.5
                    else:
                        return (math.sqrt((self.x-left_circle[3][1][0])**2 + (self.y-left_circle[3][1][1])**2) - left_circle[3][0])/40*13.5
                

fps = 50
delta_time = 1/fps
clock = pygame.time.Clock()
run = True

up_angle, down_angle, left_angle, right_angle = 0, 180, 90, -90
angle = [up_angle, down_angle, left_angle, right_angle]

poss_start = [(443, 449), (324, 42), (588, 186), (180, 305)]

start_pos = poss_start[0]
start_angle = angle[0]
images = [(street, (0,0))]
game_start = GameStart()

# path = [(286, 399), (287, 374), (288, 350), (287, 331), (298, 312), (322, 301), (371, 289), (426, 280)]
path = []

car = Car(0.5, 5, start_angle, start_pos, path)

pygame.time.set_timer(pygame.USEREVENT, 5000)

w, h = pygame.display.get_surface().get_size()

border = [(272, 40), (510, 42), (512, 141), (610, 142), (611, 380), (512, 381), (511, 481), (271, 480), (271, 383), (172, 382), (171, 143), (272, 142), (272, 40)]
line1 = [(392, 341), (391, 481)]
line2 = [(392, 181), (391, 40)]
line3 = [(471, 260), (611, 260)]
line4 = [(312, 261), (172, 260)]

turning_pts = [(287, 326), (167, 127), (329, 168), (129, 288)]

left_circle, right_circle, left_e, right_e = circle_lst()

distance = 0

while run:
    clock.tick(fps)
    
    draw(gameDisplay, images, car, game_start)
    
    while not game_start.started:
        blit_text_center(gameDisplay, font, f"Press any key to start")
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                break
            if event.type == pygame.KEYDOWN:
                game_start.start_run()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
            break
        
    d, angle = car.move(0.5)
    p = car.project()
    # car_copy = car.rect.copy()
    # car_copy.x = car.x
    # car_copy.y = car.y
    
    # if not (collideRectPolygon(car_copy, border) or collideRectPolygon(car_copy, line1) or collideRectPolygon(car_copy, line2) or collideRectPolygon(car_copy, line3) or collideRectPolygon(car_copy, line4)):
    #     car.rect = car_copy.copy()
    # else:
    #     pygame.time.wait(500)
    #     run = False
    
pygame.quit()