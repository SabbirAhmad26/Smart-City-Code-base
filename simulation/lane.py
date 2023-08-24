import pygame
import sys
import time
import math
import random
from utils import *
from car import *

pygame.font.init()

street = pygame.image.load("imgs/extension.png")
width, height = street.get_width(), street.get_height()
gameDisplay = pygame.display.set_mode((width, height))

pygame.display.set_caption("Simulation")
font = pygame.font.SysFont("comicsans", 44)

fps = 60
clock = pygame.time.Clock()
run = True

up_angle, down_angle, left_angle, right_angle = 0, 180, 90, 270
angle = [up_angle, down_angle, left_angle, right_angle]
car_up = scale_image(pygame.image.load("imgs/car.png"), 0.45)
poss_start = [(443, 449), (324, 42), (588, 186), (180, 305)]

# direction = input("Enter the initial direction: ")   #up, down, left, right
# path_dir = input("Enter the path direction: ")  #straight, left, right

# if direction == "up":
#     index = 0
# elif direction == "down":
#     index = 1
# elif direction == "left":
#     index = 2
# else:
#     index = 3

# path = State(path_dir)[index]

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
    
class Car(Car_Props):
    img = car_up
    
    def __init__(self, max_vel, rotation_vel, angle, poss, path=[]):
        super().__init__(max_vel, rotation_vel, poss)
        self.path = path
        self.current_point = 0
        self.vel = max_vel
        self.angle = angle

    def draw_points(self, win):
        for point in self.path:
            pygame.draw.circle(gameDisplay, (255, 0, 0), point, 5)
            
    def draw(self, gameDisplay):
        super().draw(gameDisplay)
        self.draw_points(gameDisplay)
            
    def calculate_angle(self):
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
        return self.angle

    def update_path_point(self):
        target = self.path[self.current_point]
        rect = pygame.Rect(
            self.x, self.y, self.img.get_width(), self.img.get_height())
        if rect.collidepoint(*target):
            self.current_point += 1

    def move(self, vel):
        if self.current_point >= len(self.path):
            return

        angle = self.calculate_angle()
        self.update_path_point()
        super().move(1)
        return angle
        
    def distance(self, prev_pos, d):
        if self.x == prev_pos.x:
            d = d + abs(self.y - prev_pos.y)
        elif self.x == prev_pos.x:
            d = d + abs(self.x - prev_pos.x)
        else:
            d = d + math.sqrt((self.x - prev_pos.x)**2 + (self.y - prev_pos.y)**2)
        return d

    def reset(self):
        if abs(self.x-self.path[-1][0]) <= 0.7 and abs(self.y-self.path[-1][1]) <= 0.7:
            return True


images = [(street, (0,0))]
game_info = GameInfo()
car = Car(1, 4, angle[3], poss_start[3])


# path1 = State('right')[0]
# car1 = Car(1,4,angle[0],poss_start[0],path1)

while run:
    clock.tick(fps)
    
    # pygame.draw.circle(gameDisplay, (255, 100, 0), (313, 294), 5)
    draw(gameDisplay, images, car, game_info)
    
    # while not game_info.started:
    #     blit_text_center(gameDisplay, font, f"Press any key to start")
    #     pygame.display.update()
    #     for event in pygame.event.get():
    #         if event.type == pygame.QUIT:
    #             run = False
    #             # pygame.quit()
    #             break

    #         if event.type == pygame.KEYDOWN:
    #             game_info.start_run()

    # for event in pygame.event.get():
    #     if event.type == pygame.QUIT:
    #         run = False
    #         break

    # angle = car.move(1)
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
            pygame.quit()
            break
        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            car.path.append(pos)
print(car.path)

pygame.quit()