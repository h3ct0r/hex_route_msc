__author__ = 'h3ct0r'

from math import sin, cos, pi, sqrt, atan2, degrees, ceil, floor
from decimal import Decimal, ROUND_CEILING, ROUND_FLOOR
import random
from shapely.geometry import *
import pygame
import json
import subprocess
import sys
from shapely.geometry import *
from shapely import affinity
import re
import heapq
import time
import datetime
import numpy as np

sys.setrecursionlimit(1500)

class Ball(object):
    ball_id = 0
    actual_pos = (0, 0)
    color = (0, 255, 0)
    radius = 9
    previous_pos = None
    waypoint_list = []
    previous_waypoint_list = []
    status = 'start'
    hexagon_counter = 0
    square_counter = 0
    hex_max = 15
    square_max = 4
    remaining_battery = 0
    battery_trajectory = []
    actual_battery_trajectory = []


    def __init__(self, ball_id, actual_pos, radius, color, waypoint_list, interval_sampling=15):
        self.ball_id = ball_id
        if actual_pos is not None:
            self.actual_pos = actual_pos
        if radius is not None:
            self.radius = radius
        if color is not None:
            self.color = color
        if waypoint_list is not None:
            self.waypoint_list = waypoint_list

        self.previous_pos = []
        self.amostral_samples = np.zeros((600, 800))
        self.amostral_counter = 0
        self.interval_sampling = interval_sampling
        self.amostral_points = []

    def update_actual_pos(self, pos):

        d, theta = points_to_vector(pos, self.actual_pos)
        self.remaining_battery -= int(d)

        if self.amostral_counter >= self.interval_sampling:
            print "Sampling at:", pos, "Value:", amostral_space[pos[1]][pos[0]]

            self.amostral_samples[pos[1]][pos[0]] = np.random.normal(amostral_space[pos[1]][pos[0]], 0.2, 1)[0]
            self.amostral_counter = 0

        # if self.remaining_battery < 0:
        #     print "Battery < 0!!"

        if len(self.previous_pos) > 0 and self.previous_pos[-1] != self.actual_pos:
            self.amostral_counter += 1

        self.previous_pos.append(self.actual_pos)
        self.actual_pos = pos
        pass

    def get_previous_pos(self):
        return self.previous_pos

    def remove_waypoint_hexagon_with_counter(self):
        self.previous_waypoint_list.append(self.waypoint_list.pop(0))

        if self.hexagon_counter >= self.hex_max:
            self.hexagon_counter = 1
        else:
            self.hexagon_counter += 1
        pass

    def remove_waypoint_hexagon_without_counter(self):
        self.previous_waypoint_list.append(self.waypoint_list.pop(0))
        pass

    def remove_waypoint_square(self):
        self.previous_waypoint_list.append(self.waypoint_list.pop(0))

        if self.square_counter >= self.square_max:
            self.square_counter = 1
        else:
            self.square_counter += 1

        pass


def points_to_vector(p1, p2):
    """
    Convert two points to a vector.
    :param p1:
    :param p2:
    :return: magnitude and angle.
    """
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    mag = sqrt(dx ** 2 + dy ** 2)
    theta = atan2(dy, dx)
    return mag, theta


def vector_components(mag, theta):
    """
    :param mag:
    :param theta:
    :return: Return x,y components
    """
    # components
    x, y = mag * cos(theta), mag * sin(theta)
    return x, y


'''
def calculate_next_movement(a, b):
    """
    Calculate the next movement given the starting point "a", and the
    ending point "b"
    :param a:
    :param b:
    :return:
    """

    d, theta = points_to_vector(a, b)
    if a == b or d < 3:
        print "Reached:", b
        return 0, 0

    print "angle", degrees(theta)

    dx = a[0] - b[0]
    dy = a[1] - b[1]

    movement_x = dx * 0.01
    movement_y = dy * 0.01

    if movement_x < 0.0:
        dec_x = Decimal(movement_x)
        dec_x = dec_x.to_integral_exact(rounding=ROUND_FLOOR)
        movement_x = int(dec_x)
    else:
        movement_x = int(ceil(movement_x))

    if movement_y < 0.0:
        dec_y = Decimal(movement_y)
        dec_y = dec_y.to_integral_exact(rounding=ROUND_FLOOR)
        movement_y = int(dec_y)
    else:
        movement_y = int(ceil(movement_y))

    return movement_x, movement_y
'''

def calculate_next_movement(a, b, speed):
    """
    Calculate the next movement given the starting point "a", and the
    ending point "b"
    :param a:
    :param b:
    :return:
    """

    d, theta = points_to_vector(a, b)
    if a == b or d < 3:
        #print "Reached:", b
        return 0, 0

    #print "d", d, "angle", degrees(theta)

    dx, dy = vector_components(speed, theta)

    movement_x = dx
    movement_y = dy

    #print "mov x:", movement_x, "mov y:", movement_y

    # if movement_x < 0.0:
    #     dec_x = Decimal(movement_x)
    #     dec_x = dec_x.to_integral_exact(rounding=ROUND_FLOOR)
    #     movement_x = int(dec_x)
    # else:
    #     movement_x = int(ceil(movement_x))
    #
    # if movement_y < 0.0:
    #     dec_y = Decimal(movement_y)
    #     dec_y = dec_y.to_integral_exact(rounding=ROUND_FLOOR)
    #     movement_y = int(dec_y)
    # else:
    #     movement_y = int(ceil(movement_y))

    return movement_x, movement_y


BLACK     = (   0,   0,   0)
WHITE     = ( 255, 255, 255)
GREEN     = (   0, 255,   0)
RED       = ( 255,   0,   0)
BLUE      = (   0,  60,   255)
YELLOW    = ( 255, 255,   0)
LIGHT_BLUE = (0, 128, 255)

robot_colors = [
    (102, 0, 0),
    (76, 153, 0),
    (0, 102, 204),
    (255, 102, 102),
    (255, 153, 51),
    (255, 102, 255),
    (160, 160, 160),
    (51, 0, 102),
    (135, 104, 106),
    (0, 200, 20),
    (205, 17, 240),
    (100, 100, 100),
    (255, 0, 0),
    (0, 255, 0)
]

IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600

pygame.init()
mono_font = pygame.font.SysFont("monospace", 12)
monoFont16 = pygame.font.SysFont("monospace", 16)
clock = pygame.time.Clock()

screen = pygame.display.set_mode((IMAGE_WIDTH, IMAGE_HEIGHT))

is_loop_active = True
is_priorize_fps = False

start_point = (0, 0)
robot_array = []
robot_square_array = []

json_string = sys.argv[1]
json_data = json.loads(json_string)
start_point = json_data['start_point']
robot_battery = json_data['battery_autonomy']
external_point_list = None
is_executed_external_point_list = False

amostral_space = np.loadtxt(json_data['amostral_space_file'])

robot_battery_trajectory = {}
if 'robot_battery_trajectory' in json_data:
    robot_battery_trajectory = json_data['robot_battery_trajectory']

color_counter = 0
for key, data in json_data['robots'].items():
    robot = Ball(key, start_point, 5, robot_colors[color_counter], data, interval_sampling=70)
    robot.hex_max = json_data['points_per_hex'] - 1
    robot.square_max = json_data['points_per_square'] + 1
    robot.remaining_battery = robot_battery
    robot_array.append(robot)
    if 'robot_battery_trajectory' in json_data:
        robot.battery_trajectory = json_data['robot_battery_trajectory'][key]
    color_counter += 1
    pass

if 'robots_square' in json_data:
    color_counter = 0
    for key, data in json_data['robots_square'].items():
        robot = Ball(key, start_point, 5, robot_colors[color_counter], data, interval_sampling=30)
        robot.hex_max = json_data['points_per_hex']
        robot.square_max = json_data['points_per_square'] + 1
        robot.remaining_battery = robot_battery
        robot_square_array.append(robot)
        color_counter += 1
        pass

if 'external_points' in json_data:
    external_point_list = json_data['external_points']

background = pygame.image.load("campo_futebol.png")
backgroundRect = background.get_rect()

time_start = datetime.datetime.now()

ignore_waypoint_counter = 0

while is_loop_active:
    screen.fill(BLACK)
    screen.blit(background, backgroundRect)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            is_loop_active = False

        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            if pygame.mouse.get_pressed()[0]:
                current_wp_a = pos
                pass
            if pygame.mouse.get_pressed()[2]:
                pass

        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_SPACE):
            pass

        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_ESCAPE):
            pass

    if len(robot_square_array) > 0:
        # Calculate new pos and draw waypoint
        for i in range(len(robot_array)):
            robot = robot_array[i]
            robot_square = robot_square_array[i]

            if len(robot.waypoint_list) <= 0:
                robot.status = 'end'

            if len(robot_square.waypoint_list) <= 0:
                robot_square.status = 'end'

            if robot.status == 'reached_stop' and robot_square.status != 'reached_stop':
                robot_square.status = 'continue'

            if robot_square.status == 'reached_stop' and robot.status != 'reached_stop':
                robot.status = 'continue'

            elif robot_square.status == 'reached_stop' and robot.status == 'reached_stop':
                robot.status = 'continue'
                robot_square.status = 'continue'

            #print "All", robot.hexagon_counter, robot_square.square_counter

            if robot.status != 'stop' and robot.status != 'end' and robot.status != 'reached_stop':
                # Hexagon
                pygame.draw.circle(screen, RED, robot.waypoint_list[0], 2)
                mov_x, mov_y = calculate_next_movement(robot.actual_pos, robot.waypoint_list[0], 2.0)
                if mov_x == 0 and mov_y == 0:
                    if len(robot.waypoint_list) > 0:
                        robot.remove_waypoint_hexagon_with_counter()
                        if robot.hexagon_counter == 1:
                            print "hex reached stop"
                            robot.status = 'reached_stop'
                    continue

                calculated_x = robot.actual_pos[0] + mov_x # it was - before
                calculated_y = robot.actual_pos[1] + mov_y
                robot.update_actual_pos((calculated_x, calculated_y))

            if robot_square.status != 'stop' and robot_square.status != 'end' and robot_square.status != 'reached_stop':
                # Square
                pygame.draw.circle(screen, YELLOW, robot_square.waypoint_list[0], 2)
                mov_x, mov_y = calculate_next_movement(robot_square.actual_pos, robot_square.waypoint_list[0], 2.0)
                if mov_x == 0 and mov_y == 0:
                    if len(robot_square.waypoint_list) > 0:
                        robot_square.remove_waypoint_square()
                        if robot_square.square_counter == 1:
                            print "square reached stop"
                            robot_square.status = 'reached_stop'
                    continue

                calculated_x = robot_square.actual_pos[0] + mov_x # it was - before
                calculated_y = robot_square.actual_pos[1] + mov_y
                robot_square.update_actual_pos((calculated_x, calculated_y))

            pass
    else:
        for i in range(len(robot_array)):
            robot = robot_array[i]

            if len(robot.waypoint_list) <= 0:
                if external_point_list != None and is_executed_external_point_list == False:
                    robot.waypoint_list = external_point_list
                    is_executed_external_point_list = True
                else:
                    robot.status = 'end'
                continue

            # ignore_waypoint_counter

            pygame.draw.circle(screen, RED, robot.waypoint_list[0], 2)
            mov_x, mov_y = calculate_next_movement(robot.actual_pos, robot.waypoint_list[0], 2.0)
            if mov_x == 0 and mov_y == 0:
                print "robot.actual_pos", robot.actual_pos
                if len(robot.waypoint_list) > 0:
                    robot.remove_waypoint_hexagon_with_counter()
                    if robot.hexagon_counter == 1:
                        print "hex reached stop"
                        robot.status = 'reached_stop'
                continue

            calculated_x = robot.actual_pos[0] + mov_x # it was - before
            calculated_y = robot.actual_pos[1] + mov_y
            robot.update_actual_pos((calculated_x, calculated_y))

    '''
    # Calculate new pos and draw waypoint
    for robot in robot_square_array:
        if len(robot.waypoint_list) <= 0:
            robot.status = 'end'
            continue

        pygame.draw.circle(screen, YELLOW, robot.waypoint_list[0], 2)

        mov_x, mov_y = calculate_next_movement(robot.actual_pos, robot.waypoint_list[0], 0.5)
        if mov_x == 0 and mov_y == 0:
            if len(robot.waypoint_list) > 0:
                robot.waypoint_list.pop(0)
            continue

        calculated_x = robot.actual_pos[0] + mov_x # it was - before
        calculated_y = robot.actual_pos[1] + mov_y
        robot.update_actual_pos((calculated_x, calculated_y))
        pass
    '''

    # Draw robots
    robot_end_counter = 0
    for robot in robot_array:
        if robot.status == 'end':
            robot_end_counter += 1

        # Draw robot history
        if not is_priorize_fps:
            for pos in robot.get_previous_pos():
                #pygame.draw.line(screen, robot.color, pos, pos)
                pygame.draw.circle(screen, robot.color, (int(pos[0]), int(pos[1])), 2)
                pass

        pygame.draw.circle(screen, robot.color, (int(robot.actual_pos[0]), int(robot.actual_pos[1])), 4)
        #label = monoFont16.render(str(int(robot.remaining_battery)), 10, WHITE)
        #screen.blit(label, robot.actual_pos)

    # Draw robots_square
    for robot in robot_square_array:

        # Draw robot history
        #'''
        for pos in robot.get_previous_pos():
            pygame.draw.line(screen, BLACK, (int(pos[0]), int(pos[1])), (int(pos[0]), int(pos[1])), 2)
            pass
        #'''

        pygame.draw.circle(screen, BLUE, (int(robot.actual_pos[0]), int(robot.actual_pos[1])), 4)
        pygame.draw.circle(screen, robot.color, (int(robot.actual_pos[0]), int(robot.actual_pos[1])), 2)
        label = monoFont16.render(str(int(robot.remaining_battery)), 10, WHITE)
        screen.blit(label, robot.actual_pos)

    if robot_end_counter == len(robot_array):
        print "Sim over"
        is_loop_active = False

    # Labels on the hexagon points
    for key, data in json_data['robots'].items():
        counter = 0
        for i in data:
            #label = mono_font.render(str(counter + 1), 1, BLACK)
            label = mono_font.render(".", 1, BLACK)
            screen.blit(label, i)
            counter += 1
            pass
        pass

    for robot in robot_array:
        label = monoFont16.render(str(int(robot.remaining_battery)), 10, WHITE)
        screen.blit(label, robot.actual_pos)

    clock.tick(50)
    pygame.display.flip()
    pygame.display.set_caption('FPS: ' + str(clock.get_fps()))

    pass

time_end = datetime.datetime.now()
passed_time = time_end - time_start


max_samplings = np.zeros((600, 800))

for robot in robot_array:
    max_samplings = np.maximum(robot.amostral_samples, max_samplings)

np.savetxt('/tmp/magnetic_sampled.np', max_samplings)

text_file = open("/tmp/timeoutput.txt", "a")
text_file.write("Time was: %s " % str(passed_time.seconds))
if 'message' in json_data:
    message = json_data['message']
    text_file.write(message)
    text_file.write(" ")
text_file.write("\n")
text_file.close()

#time.sleep(1)
pygame.quit()