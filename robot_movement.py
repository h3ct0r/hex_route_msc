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

    def __init__(self, ball_id, actual_pos, radius, color, waypoint_list):
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

    def update_actual_pos(self, pos):
        self.previous_pos.append(self.actual_pos)
        self.actual_pos = pos
        pass

    def get_previous_pos(self):
        return self.previous_pos

    def remove_waypoint_hexagon(self):
        self.previous_waypoint_list.append(self.waypoint_list.pop(0))

        if self.hexagon_counter >= 15:
            self.hexagon_counter = 1
        else:
            self.hexagon_counter += 1
        pass

    def remove_waypoint_square(self):
        self.previous_waypoint_list.append(self.waypoint_list.pop(0))

        if self.square_counter >= 5:
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
clock = pygame.time.Clock()

screen = pygame.display.set_mode((IMAGE_WIDTH, IMAGE_HEIGHT))

is_loop_active = True
is_priorize_fps = True

start_point = (0, 0)
robot_array = []
robot_square_array = []

json_string = sys.argv[1]
json_data = json.loads(json_string)
start_point = json_data['start_point']

color_counter = 0
for key, data in json_data['robots'].items():
    data.append(start_point)
    robot = Ball(key, start_point, 5, robot_colors[color_counter], data)
    robot_array.append(robot)
    color_counter += 1
    pass

if 'robots_square' in json_data:
    color_counter = 0
    for key, data in json_data['robots_square'].items():
        robot = Ball(key, start_point, 5, robot_colors[color_counter], data)
        robot_square_array.append(robot)
        color_counter += 1
        pass

'''
robot_a = Ball(1, start_point, 5, WHITE, [(100, 100), (100, 200), (200, 200), (450, 300)])
robot_array.append(robot_a)

robot_b = Ball(2, start_point, 5, BLUE, [(400, 400), (30, 32), (45, 50), (210, 240), (350, 300)])
robot_array.append(robot_b)
'''

#waypoints_a = [(20, 30), (300, 300)]
#current_wp_a = waypoints_a.pop(0)
#current_pos_a = start_point

#background = pygame.image.load("capanema800x600.png")
background = pygame.image.load("campo_futebol.png")
backgroundRect = background.get_rect()

time_start = datetime.datetime.now()

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

    # Calculate new pos and draw waypoint
    if len(robot_square_array) > 0:
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
                mov_x, mov_y = calculate_next_movement(robot.actual_pos, robot.waypoint_list[0], 2.5)
                if mov_x == 0 and mov_y == 0:
                    if len(robot.waypoint_list) > 0:
                        robot.remove_waypoint_hexagon()
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
                mov_x, mov_y = calculate_next_movement(robot_square.actual_pos, robot_square.waypoint_list[0], 0.5)
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
                robot.status = 'end'
                continue

            pygame.draw.circle(screen, RED, robot.waypoint_list[0], 2)
            mov_x, mov_y = calculate_next_movement(robot.actual_pos, robot.waypoint_list[0], 2.5)
            if mov_x == 0 and mov_y == 0:
                if len(robot.waypoint_list) > 0:
                    robot.remove_waypoint_hexagon()
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

        if not is_priorize_fps:
            for pos in robot.get_previous_pos():
                #pygame.draw.line(screen, robot.color, pos, pos)
                pygame.draw.circle(screen, robot.color, pos, 2)
                pass

        pygame.draw.circle(screen, robot.color, robot.actual_pos, 4)

    # Draw robots_square
    for robot in robot_square_array:
        for pos in robot.get_previous_pos():
            pygame.draw.line(screen, LIGHT_BLUE, pos, pos)
            pass

        pygame.draw.circle(screen, BLUE, robot.actual_pos, 4)
        pygame.draw.circle(screen, robot.color, robot.actual_pos, 2)

    if robot_end_counter == len(robot_array):
        print "Sim over"
        is_loop_active = False

    for key, data in json_data['robots'].items():
        counter = 0
        for i in data:
            label = mono_font.render(str(counter + 1), 1, BLACK)
            screen.blit(label, i)
            counter += 1
            pass
        pass

    clock.tick(50)
    pygame.display.flip()
    pygame.display.set_caption('FPS: ' + str(clock.get_fps()))

    pass

time_end = datetime.datetime.now()
passed_time = time_end - time_start

text_file = open("/tmp/timeoutput_simplemovement.txt", "w")
text_file.write("Time was: %s" % str(passed_time.seconds))
text_file.write("\n")
text_file.close()

#time.sleep(4)
pygame.quit()