__author__ = 'h3ct0r'

from math import sin, cos, pi, sqrt, atan2, degrees
import random
from shapely.geometry import *
import pygame
import json
import subprocess
import sys
from shapely.geometry import *
from shapely import affinity
from shapely import ops
import re
import heapq
import Buttons
import time
from random import shuffle
import os
import datetime

sys.setrecursionlimit(1500)

BLACK     = (   0,   0,   0)
WHITE     = ( 255, 255, 255)
GREEN     = (   0, 255,   0)
RED       = ( 255,   0,   0)
BLUE      = (   0,  60,   255)
YELLOW    = ( 255, 255,   0)
LIGHT_BLUE = (0, 128, 255)

IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600

THETA = pi / 3.0 					# Angle from one point to the next
RADIUS = 35 						# Size of a hex
#HEXES_WIDE = 7                      # How many hexes in a row
#HEXES_HIGH = 16                     # How many rows of hexes

HEXES_WIDE = int(IMAGE_WIDTH / (RADIUS)) # How many hexes in a row
HEXES_HIGH = int(IMAGE_HEIGHT / (RADIUS)) + 1 # How many rows of hexes


HALF_RADIUS = RADIUS / 2.0
HALF_HEX_HEIGHT = sqrt(RADIUS ** 2 - HALF_RADIUS ** 2)

simulation_done = False
continue_simulation = False
continue_simulation_zigzag = False
point_list = []
hex_intersection = []
adjacency_dict = {}

pygame.init()
monoFont = pygame.font.SysFont("monospace", 12)
monoFont16 = pygame.font.SysFont("monospace", 16)
monoFont20 = pygame.font.SysFont("monospace", 20)
clock = pygame.time.Clock()

screen = pygame.display.set_mode( (IMAGE_WIDTH, IMAGE_HEIGHT) )

# Point on where the robots start and end their trajectory
trajectory_start_pt = (0, 0)
ga_input_file = "/tmp/adjacency_list.gamtsp"
robot_paths = {}
robot_path_angles = {}
robot_waypoints_json = None
robot_waypoints = {}

user_preference_file = "sim_preferences.json"

number_of_robots = 1
battery_autonomy = 1000

lines_width = 10

robot_colors = {
    'a': (102, 0, 0),
    'b': (76, 153, 0),
    'c': (0, 102, 204),
    'd': (255, 102, 102),
    'e': (255, 153, 51),
    'f': (255, 102, 255),
    'g': (160, 160, 160),
    'h': (51, 0, 102),
    'i': (135, 104, 106),
    'j': (0, 200, 20),
    'k': (205, 17, 240),
    'l': (100, 100, 100),
    'm': (255, 0, 0),
    'n': (0, 255, 0)
}

rot_angle_dict = {
    "1": 30,
    "2": 90,
    "3": 150,
    "4": 210,
    "5": 270,
    "6": 330
}

hex_angle_dict = {
    "a": 0,
    "b": 60,
    "c": 120,
    "d": 180,
    "e": 240,
    "f": 300,
}

rot_angle_list = ["1", "2", "3", "4", "5", "6"]
hex_angle_list = ["a", "b", "c", "d", "e", "f"]

generated_leaflet_pos = [(281, 138), (310, 155), (319, 172), (261, 138), (256, 147), (315, 181), (310, 190), (250, 155), (245, 163), (305, 198), (300, 207), (240, 172), (249, 189), (280, 207), (260, 207), (259, 206)]
generated_square_pos = [(261, 157), (261, 193), (297, 193), (297, 157)]

generated_leaflet_pos_battery = 0

print "HEXES_WIDE", HEXES_WIDE, "HEXES_HIGH", HEXES_HIGH


def generate_json_string_from_list(my_list):
    point_str = "["
    for x in xrange(len(my_list)):
        point = my_list[x]
        point_str += "[" + str(int(point[0])) + "," + str(int(point[1])) + "]"
        if x < len(my_list) - 1:
            point_str += ","
    point_str += "]"
    return point_str


def shortest_path(G, start, end):
    def flatten(L):       # Flatten linked list of form [0,[1,[2,[]]]]
        while len(L) > 0:
            yield L[0]
            L = L[1]

    q = [(0, start, ())]  # Heap of (cost, path_head, path_rest).
    visited = set()       # Visited vertices.
    while True:
        (cost, v1, path) = heapq.heappop(q)
        if v1 not in visited:
            visited.add(v1)
            if v1 == end:
                return list(flatten(path))[::-1] + [v1]
            path = (v1, path)
            for (v2, cost2) in G[v1].iteritems():
                if v2 not in visited:
                    heapq.heappush(q, (cost + cost2, v2, path))


def dijkstra(graph, src, dest, visited=[], distances={}, predecessors={}):
    """
    calculates a shortest path tree routed in src
    """
    # a few sanity checks
    if src not in graph:
        raise TypeError('the root of the shortest path tree cannot be found in the graph')
    if dest not in graph:
        raise TypeError('the target of the shortest path cannot be found in the graph')
        # ending condition
    if src == dest:
        # We build the shortest path and display it
        path = []
        pred = dest
        while pred != None:
            path.append(pred)
            pred = predecessors.get(pred, None)
        #print('shortest path: ' + str(path) + " cost=" + str(distances[dest]))
        return path, distances[dest]
    else:
        # if it is the initial  run, initializes the cost
        if not visited:
            distances[src] = 0
        # visit the neighbors
        for neighbor in graph[src]:
            if neighbor not in visited:

                #if src not in distances:
                #    distances[src] = 0

                new_distance = distances[src] + graph[src][neighbor]
                if new_distance < distances.get(neighbor, float('inf')):
                    distances[neighbor] = new_distance
                    predecessors[neighbor] = src

        # mark as visited
        visited.append(src)
        # now that all neighbors have been visited: recurse
        # select the non visited node with lowest distance 'x'
        # run Dijskstra with src='x'
        unvisited = {}
        for k in graph:
            if k not in visited:
                unvisited[k] = distances.get(k, float('inf'))
        x = min(unvisited, key=unvisited.get)
        path, distances[dest] = dijkstra(graph, x, dest, visited, distances, predecessors)
        return path, distances[dest]


def hex_points(x, y):
    '''Given x and y of the origin, return the six points around the origin of RADIUS distance'''
    for i in range(6):
        yield cos(THETA * i) * RADIUS + x, sin(THETA * i) * RADIUS + y


def hex_centres():
    for x in range(HEXES_WIDE):
        for y in range(HEXES_HIGH):
            yield (x * 3 + 1) * RADIUS + RADIUS * 1.5 * (y % 2), (y + 1) * HALF_HEX_HEIGHT


def get_random_colour():
    return (random.randrange(256),random.randrange(256),random.randrange(256))


def get_hex_point_list():
    ''' Return a list of Shapely hex polygons '''
    poly_list = []

    for x,y in hex_centres():
        poly_list.append(Polygon( list(hex_points(x,y)) ))

    return poly_list


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


def is_number(s):
    """
    Return is the element is a number
    :param s:
    :return:
    """
    try:
        float(s)
        return True
    except ValueError:
        return False


def generate_cover_poly_with_translation(hex_poly_obj, hex_line_angle_obj, c_center):
    cx2 = int(hex_poly_obj.centroid.x)
    cy2 = int(hex_poly_obj.centroid.y)

    cover_poly_obj = Polygon(generated_leaflet_pos)
    cover_poly_obj = affinity.translate(cover_poly_obj, cx2 - c_center[0], cy2 - c_center[1])
    cover_poly_obj = affinity.rotate(cover_poly_obj, hex_line_angle_obj)

    return cover_poly_obj

# Jarvis algorithm
TURN_LEFT, TURN_RIGHT, TURN_NONE = (1, -1, 0)

def turn(p, q, r):
    """Returns -1, 0, 1 if p,q,r forms a right, straight, or left turn."""
    return cmp((q[0] - p[0])*(r[1] - p[1]) - (r[0] - p[0])*(q[1] - p[1]), 0)

def _dist(p, q):
    """Returns the squared Euclidean distance between p and q."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    return dx * dx + dy * dy

def _next_hull_pt(points, p):
    """Returns the next point on the convex hull in CCW from p."""
    q = p
    for r in points:
        t = turn(p, q, r)
        if t == TURN_RIGHT or t == TURN_NONE and _dist(p, r) > _dist(p, q):
            q = r
    return q

def convex_hull(points):
    """Returns the points on the convex hull of points in CCW order."""
    hull = [min(points)]
    for p in hull:
        q = _next_hull_pt(points, p)
        if q != hull[0]:
            hull.append(q)
    return hull


def is_number(s):
    """
    Return is the element is a number
    :param s:
    :return:
    """

    try:
        float(s)
        return True
    except ValueError:
        return False

def evaluate(chromosome, points, distances, start_point, adjacency):

    print 'Starting evaluations...'

    robot_nodes = {}
    actual_robot_letter = ''
    for elem in chromosome:
        if is_number(elem):
            robot_nodes[actual_robot_letter].append(elem)
        else:
            actual_robot_letter = elem
            robot_nodes[actual_robot_letter] = []

    # Calculate the costs of the paths
    cost_list = []
    min_path = -1
    max_path = -1

    total_costs = []
    cross_itself_paths = 0
    cross_different_paths = 0
    robot_paths = []

    print "ROBOT NODES:", robot_nodes

    for key, value in robot_nodes.items():
        if len(value) <= 0:
            min_path = 0
            continue

        local_cost_list = []
        local_points = []
        for j in range(len(value)):
            local_points.append(points[str(value[j])])
            if j + 1 < len(value) - 1:
                local_cost_list.append(distances[str(value[j])][str(value[j+1])])

        # Start to pos and pos to end
        #local_cost_list.append(distances[str(start_point)][str(value[0])])
        d, tetha = points_to_vector(start_point, points[value[0]])
        local_cost_list.append(d)

        #local_cost_list.append(distances[str(value[-1])][str(start_point)])
        d, tetha = points_to_vector(start_point, points[value[-1]])
        local_cost_list.append(d)

        min_local = min(local_cost_list)
        if min_local < min_path or min_path == -1:
            min_path = min_local

        max_local = max(local_cost_list)
        if max_local > max_path or max_path == -1:
            max_path = max_local

        cost_list = cost_list + local_cost_list
        total_costs.append(sum(local_cost_list))
        #print "TC", total_costs
        #print local_points

        robot_paths.append(local_points)

        if len(local_points) > 2:
            ring = LinearRing(local_points)
            if not ring.is_valid:
                cross_itself_paths += 1

    # Check for closeness of neighbours
    print "adjacency", adjacency

    distant_neighbours = 0
    for key, value in robot_nodes.items():
        if len(value) <= 0:
            continue

        for h in range(len(value)):
            if h + 1 < len(value):

                #print "value[h + 1]", value[h + 1]
                #print "value[h]", value[h]
                #print "adjacency.items()", adjacency.keys()
                #print "adjacency[value[h]]", adjacency[int(value[h]) - 1]

                if int(value[h + 1]) - 1 not in adjacency[int(value[h]) - 1]:
                    distant_neighbours += 1

    sum_paths = sum(cost_list)

    # Heuristics to minimize the distance differences between the robots
    dif_distances = 0
    if len(total_costs) > 1:
        for index1 in range(len(total_costs)):
            cost = total_costs[index1]
            for index2 in range(len(total_costs)):
                cost2 = total_costs[index2]
                dif_distances += abs(cost2 - cost)
    else:
        dif_distances = total_costs[0]

    fitness = (sum_paths) + (((max_path - min_path) + (dif_distances)) * 2) + (cross_itself_paths * 200) + (distant_neighbours * 100)

    print 'Evaluation finished...', fitness
    return fitness

hex_list = get_hex_point_list()

distances_dict = {}

#background = pygame.image.load("capanema800x600.png")
background = pygame.image.load("campo_futebol.png")
backgroundRect = background.get_rect()

button_up = Buttons.Button()
button_down = Buttons.Button()

button_more_size = Buttons.Button()
button_less_size = Buttons.Button()

button_more_battery = Buttons.Button()
button_less_battery = Buttons.Button()

button_bigger_width_lines = Buttons.Button()
button_less_width_lines = Buttons.Button()

button_change_direction = Buttons.Button()

button_change_division_algorithm = Buttons.Button()

button_load_saved_points = Buttons.Button()
button_save_points = Buttons.Button()

poly_union = None

user_selected_angle = 0
user_selected_algo = 2
user_selected_algo_description = ['ksplit', 'GA', 'Kmeans', 'Division']

redraw = True

while simulation_done == False:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            redraw = True
            simulation_done = True

        elif event.type == pygame.MOUSEBUTTONDOWN:
            redraw = True
            pos = pygame.mouse.get_pos()

            if button_up.pressed(pos):
                if number_of_robots < 10:
                    number_of_robots += 1
                    break

            if button_down.pressed(pos):
                if number_of_robots > 1:
                    number_of_robots -= 1
                    break

            if button_more_size.pressed(pos):
                if RADIUS <= 95:
                    RADIUS += 5
                    HEXES_WIDE = int(IMAGE_WIDTH / (RADIUS)) # How many hexes in a row
                    HEXES_HIGH = int(IMAGE_HEIGHT / (RADIUS)) + 1 # How many rows of hexes
                    HALF_RADIUS = RADIUS / 2.0
                    HALF_HEX_HEIGHT = sqrt(RADIUS ** 2 - HALF_RADIUS ** 2)
                    hex_list = get_hex_point_list()
                    print "HEXES_WIDE", HEXES_WIDE, "HEXES_HIGH", HEXES_HIGH
                    hex_intersection = []
                    robot_paths = {}
                    robot_path_angles = {}
                    break

            if button_less_size.pressed(pos):
                if RADIUS > 6:
                    RADIUS -= 5
                    HEXES_WIDE = int(IMAGE_WIDTH / (RADIUS)) # How many hexes in a row
                    HEXES_HIGH = int(IMAGE_HEIGHT / (RADIUS)) + 1 # How many rows of hexes
                    HALF_RADIUS = RADIUS / 2.0
                    HALF_HEX_HEIGHT = sqrt(RADIUS ** 2 - HALF_RADIUS ** 2)
                    hex_list = get_hex_point_list()
                    print "HEXES_WIDE", HEXES_WIDE, "HEXES_HIGH", HEXES_HIGH
                    hex_intersection = []
                    robot_paths = {}
                    robot_path_angles = {}
                    break

            if button_more_battery.pressed(pos):
                battery_autonomy += 50
                robot_waypoints['battery_autonomy'] = battery_autonomy
                break
                pass

            if button_less_battery.pressed(pos):
                battery_autonomy -= 50
                robot_waypoints['battery_autonomy'] = battery_autonomy
                break
                pass

            if button_bigger_width_lines.pressed(pos):
                lines_width += 1
                break
                pass

            if button_less_width_lines.pressed(pos):
                lines_width -= 1
                break
                pass

            if button_change_direction.pressed(pos):
                user_selected_angle += 1
                if user_selected_angle > 8:
                    user_selected_angle = 0
                break

            if button_change_division_algorithm.pressed(pos):
                user_selected_algo += 1
                if user_selected_algo > 3:
                    user_selected_algo = 0
                break

            if button_load_saved_points.pressed(pos):
                user_home = os.path.expanduser('~')
                full_pref_path = user_home + "/" + user_preference_file
                if os.path.isfile(full_pref_path):
                    with open(full_pref_path, 'r') as f:
                        data = json.load(f)
                        hex_list = get_hex_point_list()
                        hex_intersection = []
                        robot_paths = {}
                        robot_path_angles = {}
                        point_list = data["point_list"]
                        trajectory_start_pt = data["trajectory_start_pt"]

                        print "Data loaded from :", full_pref_path
                break

            if button_save_points.pressed(pos):
                user_home = os.path.expanduser('~')
                full_pref_path = user_home + "/" + user_preference_file

                if len(point_list) <= 0 or trajectory_start_pt == (0,0):
                    print "Cannot save point list empty or start point not defined"
                    break

                data = {
                   'point_list' : point_list,
                   'trajectory_start_pt' : trajectory_start_pt
                }

                with open(full_pref_path, 'w') as f:
                    json.dump(data, f)

                print "Data saved to :", full_pref_path
                break

            if pygame.mouse.get_pressed()[0]:
                trajectory_start_pt = pos
                robot_waypoints['start_point'] = trajectory_start_pt
                pass

            if pygame.mouse.get_pressed()[2]:
                found = False
                for border in point_list:
                    d, theta = points_to_vector( (pos[0], pos[1]), border )
                    if d < 10:
                        found = True
                        point_list.append( border )
                        break

                if not found:
                    point_list.append( (pos[0], pos[1]) )

                print "Left Click", pos[0], pos[1]


        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_SPACE):
            redraw = True
            continue_simulation = not continue_simulation
            hex_intersection = []

        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_PERIOD):
            redraw = True
            continue_simulation_zigzag = not continue_simulation_zigzag

        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_COMMA):
            redraw = True
            for i in hex_list[HEXES_HIGH + 1].exterior.coords:
                print "(", int(i[0]), ",", (i[1]), "),"

        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_1):
            redraw = True
            new_robot_waypoints = robot_waypoints.copy()
            del new_robot_waypoints['robots_square']
            del new_robot_waypoints['external_points']

            new_robot_waypoints["message"] = "lines_width:" + str(lines_width) + " battery:" + str(battery_autonomy) \
                                             + " hex_size:" + str(RADIUS) + " number_of_robots:" + str(number_of_robots)

            new_robot_waypoints_json = json.dumps(new_robot_waypoints)

            print "new_robot_waypoints_json:", new_robot_waypoints_json

            #p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/hex_manual_astar/robot_movement_multi_battery.py", new_robot_waypoints_json], stdout=subprocess.PIPE)
            #out, err = p.communicate()
            #outSplit = out.split()
            p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/hex_manual_astar/robot_movement_multi_battery.py", new_robot_waypoints_json], stdin=None, stdout=None, stderr=None)

            # for i in xrange(10):
            #     p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/hex_manual_astar/robot_movement_multi_battery.py", new_robot_waypoints_json], stdout=subprocess.PIPE)
            #     out, err = p.communicate()
            #     outSplit = out.split()

            pass

        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_2):
            redraw = True
            p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/hex_manual_astar/robot_movement_multi_battery.py", robot_waypoints_json], stdout=subprocess.PIPE)
            out, err = p.communicate()
            outSplit = out.split()
            pass

        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_3):
            redraw = True
            new_robot_waypoints = robot_waypoints.copy()
            del new_robot_waypoints['robots_square']

            new_robot_waypoints_json = json.dumps(new_robot_waypoints)

            print "new_robot_waypoints_json:", new_robot_waypoints_json

            p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/hex_manual_astar/robot_movement_multi_battery.py", new_robot_waypoints_json], stdout=subprocess.PIPE)
            out, err = p.communicate()
            outSplit = out.split()
            pass
        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_4):
            redraw = True
            new_robot_waypoints = robot_waypoints.copy()
            new_robot_waypoints['amostral_space_file'] = '/tmp/magnetic_white_noise.np'
            new_robot_waypoints_json = json.dumps(new_robot_waypoints)

            print "new_robot_waypoints_json:", new_robot_waypoints_json

            p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/hex_manual_astar/robot_movement_with_sampling_gaussian.py", new_robot_waypoints_json], stdout=subprocess.PIPE)
            out, err = p.communicate()
            outSplit = out.split()
            pass

        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_ESCAPE):
            redraw = True
            if len(point_list) > 0:
                point_list.pop()
            else:
                print "No more points to pop"

        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_DOWN):
            # Move down the object
            redraw = True
            for i in xrange(len(point_list)):
                point = point_list[i]
                shape_point = Point(point)
                shape_point = affinity.translate(shape_point, 0, 2)
                point_list[i] = (int(shape_point.coords[0][0]), int(shape_point.coords[0][1]))
            pass

        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_UP):
            # Move up the object
            redraw = True
            for i in xrange(len(point_list)):
                point = point_list[i]
                shape_point = Point(point)
                shape_point = affinity.translate(shape_point, 0, -2)
                point_list[i] = (int(shape_point.coords[0][0]), int(shape_point.coords[0][1]))
            pass

        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_LEFT):
            # Move left the object
            redraw = True
            for i in xrange(len(point_list)):
                point = point_list[i]
                shape_point = Point(point)
                shape_point = affinity.translate(shape_point, -2, 0)
                point_list[i] = (int(shape_point.coords[0][0]), int(shape_point.coords[0][1]))
            pass

        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_RIGHT):
            # Move right the object
            redraw = True
            for i in xrange(len(point_list)):
                point = point_list[i]
                shape_point = Point(point)
                shape_point = affinity.translate(shape_point, 2, 0)
                point_list[i] = (int(shape_point.coords[0][0]), int(shape_point.coords[0][1]))
            pass

        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_b):
            # Battery algorithm come home

            new_robot_waypoints = robot_waypoints.copy()
            del new_robot_waypoints['robots_square']
            new_robot_waypoints['robot_battery_trajectory'] = {}

            print "\n*** Algorithm to travel with battery ***"
            print "Generated leaflet pos battery    :", generated_leaflet_pos_battery
            print "Battery autonomy                 :", battery_autonomy
            print "Generated leaflet pos            :", generated_leaflet_pos

            error_found = False

            for key, value in robot_path_angles.items():
                path_list = robot_paths[key]

                if error_found:
                    break

                total_paths = []
                actual_path = []
                cover_center = (int(hex_list[HEXES_HIGH + 1].centroid.x), int(hex_list[HEXES_HIGH + 1].centroid.y))
                local_robot_battery = battery_autonomy
                pos_index = 0

                print "For robot ", key, "total path list:", path_list

                while pos_index < len(path_list):

                    hex_poly = hex_list[hex_intersection[int(path_list[pos_index]) - 1]]
                    hex_line_angle = hex_angle_dict[value[pos_index]]
                    cover_poly = generate_cover_poly_with_translation(hex_poly, hex_line_angle, cover_center)

                    start_pos = cover_poly.exterior.coords[0]
                    end_pos = cover_poly.exterior.coords[-1]

                    d1, theta = points_to_vector(trajectory_start_pt, start_pos)
                    d1 += generated_leaflet_pos_battery

                    d2, theta = points_to_vector(end_pos, trajectory_start_pt)

                    #print "d1:", d1, "local_robot_battery:", local_robot_battery, "trajectory_start_pt", trajectory_start_pt, "start_pos", start_pos, "end_pos", end_pos, "hex_line_angle", hex_line_angle, "hex_poly", hex_poly, "cover_center", cover_center

                    if d1 + d2 > local_robot_battery:
                        print "The path cannot be done."
                        error_found = True
                        total_paths.append("error")
                        actual_path = []
                        break

                    actual_path.append(int(path_list[pos_index]) - 1)
                    local_robot_battery -= d1
                    d_accumulated = d1
                    last_pos = end_pos

                    print "11 actual_path", actual_path, "total_paths", total_paths, "pos_index", pos_index

                    if int(pos_index) + 1 < len(path_list):
                        #print "enter1"
                        next_pos = pos_index + 1

                        while next_pos < len(path_list):

                            hex_poly2 = hex_list[hex_intersection[int(path_list[next_pos]) - 1]]
                            next_hex_line_angle = hex_angle_dict[value[next_pos]]
                            cover_poly2 = generate_cover_poly_with_translation(hex_poly2, next_hex_line_angle, cover_center)

                            start_pos2 = cover_poly2.exterior.coords[0]
                            end_pos2 = cover_poly2.exterior.coords[-1]

                            d3, theta = points_to_vector(last_pos, start_pos2)
                            d3 += generated_leaflet_pos_battery

                            d4, theta = points_to_vector(end_pos2, trajectory_start_pt)

                            #print "d3 + d4:", d3 + d4, "local_robot_battery:", local_robot_battery

                            if d3 + d4 <= local_robot_battery:
                                #print "111"
                                local_robot_battery -= d3
                                d_accumulated += d3
                                actual_path.append(int(path_list[next_pos]) - 1)
                                next_pos += 1
                                last_pos = end_pos2
                            else:
                                #total_paths.append(actual_path)
                                #actual_path = []
                                #actual_path.append(int(path_list[next_pos]) - 1)
                                #total_paths.append(actual_path)
                                #actual_path = []
                                print "break"
                                break

                        pos_index = next_pos
                    else:
                        #print "enter2"
                        #actual_path.append(int(path_list[pos_index]) - 1)
                        pos_index += 1

                    print "22 actual_path", actual_path, "total_paths", total_paths, "pos_index", pos_index

                    local_robot_battery = battery_autonomy
                    total_paths.append(actual_path)
                    actual_path = []

                    #print "pos_index:", pos_index, "local_robot_battery:", local_robot_battery
                pass

                print "For robot ", key, "the paths are:", total_paths
                new_robot_waypoints['robot_battery_trajectory'][key] = total_paths
            pass

            if not error_found:
                new_robot_waypoints['message'] = "battery:" + str(battery_autonomy) + " radius:" + str(RADIUS)
                new_robot_waypoints_json = json.dumps(new_robot_waypoints)

                print new_robot_waypoints_json

                # Does not allow forking, this wait for the process to end
                #p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/hex_manual_astar/robot_movement_multi_battery.py", new_robot_waypoints_json], stdout=subprocess.PIPE)
                #out, err = p.communicate()
                #outSplit = out.split()

                # Allowing forking
                p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/hex_manual_astar/robot_movement_multi_battery.py", new_robot_waypoints_json], stdin=None, stdout=None, stderr=None)
            pass


    if redraw:

        screen.fill(BLACK)
        screen.blit(background, backgroundRect)

        #Parameters:                                surface, color,       x,   y,   length, height, width,    text, text_color
        button_less_width_lines.create_button(      screen, (107,142,35), 0,    0,  50,    40,    0,        "-WIDTH", (255,255,255))
        button_bigger_width_lines.create_button(    screen, (107,142,35), 50,   0,  50,    40,    0,        "+WIDTH", (255,255,255))

        button_less_battery.create_button(  screen, (107,142,35), 130, 0, 50,    40,    0,        "-BATT", (255,255,255))
        button_more_battery.create_button(  screen, (107,142,35), 180, 0, 50,    40,    0,        "+BATT", (255,255,255))

        button_less_size.create_button(  screen, (107,142,35), 280, 0, 50,    40,    0,        "-HEX", (255,255,255))
        button_more_size.create_button(    screen, (107,142,35), 330, 0, 50,    40,    0,        "+HEX", (255,255,255))

        #Parameters:                surface, color,       x,   y,   length, height, width,    text, text_color
        button_down.create_button(                  screen, (107,142,35), 410, 0,   50,     40,     0,        "-R", (255,255,255))
        button_up.create_button(                    screen, (107,142,35), 460, 0,   50,     40,     0,        "+R", (255,255,255))

        button_change_direction.create_button(
                                                    screen, (107,142,35), 540, 0, 50,    40,    0,        "ANGLE", (255,255,255))

        button_change_division_algorithm.create_button(
                                                    screen, (107,142,35), 620,      0,  50,    40,    0,        "ALGO", (255,255,255))

        button_load_saved_points.create_button(     screen, (107,142,35), 620,    560,  50,    20,    0,        "LOAD", (255,255,255))
        button_save_points.create_button(           screen, (107,142,35), 620,    580,  50,    20,    0,        "SAVE", (255,255,255))

        if continue_simulation:
            for step_position in range(1):
                hex_intersection = []
                hex_intersection_map = {}

                counter = 1
                if len(point_list) > 1 and point_list[0] == point_list[-1]:
                    border_polygon = Polygon(point_list)
                    for i in xrange(len(hex_list)):
                        hex_poly = hex_list[i]
                        if hex_poly.within(border_polygon) or hex_poly.intersects(border_polygon):
                            hex_intersection.append(i)
                            hex_intersection_map[i] = counter
                            counter += 1

                if user_selected_angle == 0:
                    hex_angle_dict = {
                        "a": 0,
                        "b": 60,
                        "c": 120,
                        "d": 180,
                        "e": 240,
                        "f": 300,
                    }
                else:
                    hex_angle_dict = {
                        "a": 0,
                        "b": 60,
                        "c": 120,
                        "d": 180,
                        "e": 240,
                        "f": 300,
                        "g": 45,
                        "h": 90,
                        "i": 135,
                        "j": 180,
                        "k": 225,
                        "l": 270,
                        "m": 315,
                        "n": 360,
                        #"g": 90,
                        #"h": 180,
                        #"i": 270
                    }

                adjacency_dict = {}
                adjacency_dict_1toN = {}
                adjacency_dict_0toN = {}
                for i in hex_intersection:
                    adjacency_dict[i] = []
                    adjacency_dict_1toN[hex_intersection_map[i]] = []
                    adjacency_dict_0toN[hex_intersection_map[i] - 1] = []
                    poly_a = hex_list[i]
                    for j in hex_intersection:
                        if i == j:
                            continue
                        poly_b = hex_list[j]

                        if int(poly_a.distance(poly_b)) <= 1:
                            adjacency_dict[i].append(j)
                            adjacency_dict_1toN[hex_intersection_map[i]].append(hex_intersection_map[j])
                            adjacency_dict_0toN[hex_intersection_map[i] - 1].append(hex_intersection_map[j] - 1)

                print "Hex intersected :", len(hex_intersection), hex_intersection
                '''
                hex_positions = {}
                hex_positions[str(-1)] = trajectory_start_pt
                for index in hex_intersection:
                    hex_poly = hex_list[index]
                    coords = hex_poly.exterior.coords
                    center = (int(hex_poly.centroid.x), int(hex_poly.centroid.y))
                    hex_positions[str(hex_intersection_map[index] - 1)] = center
                    print hex_intersection_map[index] - 1, center
                '''

                # Positions starting with 0 to n
                hex_positions = {}
                hex_positions[str(0)] = trajectory_start_pt
                for index in hex_intersection:
                    hex_poly = hex_list[index]
                    coords = hex_poly.exterior.coords
                    center = (int(hex_poly.centroid.x), int(hex_poly.centroid.y))
                    hex_positions[str(hex_intersection_map[index])] = center
                    print hex_intersection_map[index], center

                #print adjacency_dict
                print adjacency_dict_0toN
                #print adjacency_dict_1toN
                print hex_positions

                ga_object = {}
                ga_object['distances'] = hex_positions
                ga_object['adjacency'] = adjacency_dict_0toN
                #ga_object['start_point'] = '-1'
                ga_object['start_point'] = '0'

                json_ga_object = json.dumps(ga_object)
                print json_ga_object

                f = open(ga_input_file, "w")
                f.write(str(json_ga_object))
                f.close()

                ga_config_file_string = """<ga_mtsp>
                    <io>
                        <output-dir value="testdir_xmlfile" />
                    </io>

                    <parameters>
                        <robot-number value='{0}' />
                        <popsize value='200' />
                        <elitism value='true' />
                        <tournament-size value='10' />
                        <p-crossover value='.9' />
                        <p-mutation value='.03' />
                        <generations value='500' />
                    </parameters>

                    <externals>
                        <random-seed value='29' />
                        <repetitions value='1' />
                    </externals>
                </ga_mtsp>""".format(number_of_robots)

                text_file = open("/tmp/ga_mtsp_config_file.xml", "w")
                text_file.write(ga_config_file_string)
                text_file.close()

                start_time = datetime.datetime.now()

                if user_selected_algo == 0:
                    # KSPLIT_TOUR ------------------------------
                    p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/hex_manual_astar/tsp_ksplit_tour_textmode.py",
                                          '/tmp/adjacency_list.gamtsp', str(number_of_robots), '/tmp/sol.mtsp'], stdout=subprocess.PIPE)
                    pass
                elif user_selected_algo == 1:
                    # GENETIC_ALGORITHM ------------------------
                    # run.py 'configs/config.xml' '/tmp/adjacency_list.gamtsp' '/tmp/sol.mtsp'
                    p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/ga_mtsp/run.py", '/tmp/ga_mtsp_config_file.xml',
                                          '/tmp/adjacency_list.gamtsp', '/tmp/sol.mtsp'], stdout=subprocess.PIPE)
                    pass
                elif user_selected_algo == 2:
                    # KMEANS -----------------------------------
                    p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/hex_manual_astar/kmeans_toursplit_textmode.py",
                                          '/tmp/adjacency_list.gamtsp', str(number_of_robots), '/tmp/sol.mtsp'], stdout=subprocess.PIPE)
                    pass
                else:
                    # SIMPLE DIVISION
                    p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/hex_manual_astar/simple_division_toursplit_textmode.py",
                                      '/tmp/adjacency_list.gamtsp', str(number_of_robots), '/tmp/sol.mtsp'], stdout=subprocess.PIPE)
                    pass

                out, err = p.communicate()
                outSplit = out.split()
                print "RESULT", outSplit

                robot_paths = {}
                robot_paths_array = []
                ins = open('/tmp/sol.mtsp', "r")
                for line in ins:
                    elements = line.split()
                    robot_paths_array = list(elements)
                    print "elements:", elements
                    actual_robot_letter = ''
                    for elem in elements:
                        if elem != ga_object['start_point']:
                            if is_number(elem):
                                robot_paths[actual_robot_letter].append(elem)
                            else:
                                actual_robot_letter = elem
                                robot_paths[actual_robot_letter] = []

                ins.close()

                end_time = datetime.datetime.now()
                calculated_time = end_time - start_time

                print "Robot trajectories:", robot_paths
                print "hex_list", len(hex_list)
                print "hex_intersection", hex_intersection
                print "number of robots", number_of_robots

                # Calculate the coverage points

                #generated_leaflet_pos
                #generated_square_pos

                coverage_path_points = "["
                for i in xrange(len(hex_list[HEXES_HIGH + 1].exterior.coords)):
                    point = hex_list[HEXES_HIGH + 1].exterior.coords[i]
                    coverage_path_points += "[" + str(int(point[0])) + "," + str(int(point[1])) + "]"
                    if i < len(hex_list[HEXES_HIGH + 1].exterior.coords) - 1:
                        coverage_path_points += ","
                coverage_path_points += "]"

                print "coverage_path_points", coverage_path_points

                p = subprocess.Popen(["python",
                                      "/Users/h3ct0r/PycharmProjects/hex_manual_astar/coverage_path_verticalseg.py",
                                      coverage_path_points,
                                      str(lines_width)], stdout=subprocess.PIPE)

                out, err = p.communicate()
                outSplit = out.split()
                print "RESULT", outSplit

                generated_leaflet_pos = []
                ins = open('/tmp/hex_coords_calculated.txt', "r")
                for line in ins:
                    elements = line.split()
                    generated_leaflet_pos.append((int(elements[0]), int(elements[1])))

                generated_square_pos = []
                ins = open('/tmp/square_coords_calculated.txt', "r")
                for line in ins:
                    elements = line.split()
                    generated_square_pos.append((int(elements[0]), int(elements[1])))

                ins.close()

                print "generated_leaflet_pos len:", len(generated_leaflet_pos)
                print "generated_square_pos len:", len(generated_square_pos)

                #generated_leaflet_pos_battery
                generated_leaflet_pos_battery = 0
                for i in xrange(1, len(generated_leaflet_pos)):
                    pos = generated_leaflet_pos[i]
                    pos2 = generated_leaflet_pos[i - 1]
                    d, tetha = points_to_vector(pos, pos2)
                    generated_leaflet_pos_battery += d
                    pass

                print "generated_leaflet_pos_battery:", generated_leaflet_pos_battery

                robot_waypoints = {}
                robot_waypoints['robots'] = {}
                robot_waypoints['robots_square'] = {}

                robot_path_angles = {}
                for key_robot, value in robot_paths.items():

                    print "value", value

                    list_hex_pos = ["-1"]
                    counter = 10

                    for i in range(len(value)):
                        #poly_a = hex_list[hex_intersection[int(value[i])]]
                        poly_a = hex_list[hex_intersection[int(value[i]) - 1]]

                        if i + 1 < len(value):
                            #poly_b = hex_list[hex_intersection[int(value[i + 1])]]
                            poly_b = hex_list[hex_intersection[int(value[i + 1]) - 1]]

                            d, theta = points_to_vector((poly_a.centroid.x, poly_a.centroid.y), (poly_b.centroid.x, poly_b.centroid.y))
                            degrees_theta = degrees(theta)
                            if degrees_theta < 0:
                                degrees_theta += 360

                            # 70 is the distance of nearby hexagons
                            #if d <= 70:
                            if False:
                                for j in rot_angle_dict.keys():
                                    print rot_angle_dict[j], int(degrees_theta)
                                    if rot_angle_dict[j] == int(degrees_theta) or abs(rot_angle_dict[j] - int(degrees_theta)) < 5:
                                        list_hex_pos.append(j)
                                        break
                                    pass
                            else:
                                # Test all the possibilities in this angle and distance
                                print "Generating combinations, long hexagon"

                                cx1 = int(poly_a.centroid.x)
                                cy1 = int(poly_a.centroid.y)

                                cover_poly1 = Polygon(generated_leaflet_pos).buffer(0)
                                cover_center1 = (int(cover_poly1.centroid.x) - 10, int(cover_poly1.centroid.y) - 2)

                                cover_poly1 = Polygon(generated_leaflet_pos)
                                cover_poly1 = affinity.translate(cover_poly1, cx1 - cover_center1[0], cy1 - cover_center1[1])

                                cx2 = int(poly_b.centroid.x)
                                cy2 = int(poly_b.centroid.y)

                                cover_poly2 = Polygon(generated_leaflet_pos).buffer(0)
                                cover_center2 = (int(cover_poly2.centroid.x) - 10, int(cover_poly2.centroid.y) - 2)

                                cover_poly2 = Polygon(generated_leaflet_pos)
                                cover_poly2 = affinity.translate(cover_poly2, cx2 - cover_center2[0], cy2 - cover_center2[1])

                                for i1 in hex_angle_list:
                                    angle1 = hex_angle_dict[i1]
                                    poly_rotated1 = affinity.rotate(cover_poly1, angle1)
                                    for i2 in hex_angle_list:
                                        angle2 = hex_angle_dict[i2]
                                        poly_rotated2 = affinity.rotate(cover_poly2, angle2)

                                        d, tetha = points_to_vector(poly_rotated1.exterior.coords[-2], poly_rotated2.exterior.coords[0])
                                        if i1 not in distances_dict:
                                            distances_dict[i1] = {}

                                        if str(counter) not in distances_dict[i1]:
                                            distances_dict[i1][str(counter)] = {}

                                        distances_dict[i1][str(counter)][i2] = d
                                        pass
                                pass

                                list_hex_pos.append(str(counter))
                                counter += 1

                                pass

                            pass

                    graph = {}
                    graph[str(0)] = {}
                    counter = 0
                    next_pos = str(0)

                    print "list_hex_pos", list_hex_pos
                    print "distances_dict", distances_dict

                    for h in range(len(list_hex_pos)):

                        if h + 1 < len(list_hex_pos):
                            old_pos = str(list_hex_pos[h])
                            next_pos = str(list_hex_pos[h + 1])

                            if h == 0:
                                for elem in hex_angle_list:
                                    for elem3 in hex_angle_list:
                                        key = str(h) + str(elem) + next_pos + str(elem3)
                                        #print "from", str(h), "to", key, "w:", distances_dict[elem][next_pos][elem3]
                                        #print "graph[str(h)]", graph[str(h)]
                                        #print "elem", elem, "next_pos", next_pos, "elem3", elem3
                                        #print "distances_dict[elem][next_pos][elem3]", distances_dict[elem][next_pos][elem3]
                                        graph[str(h)][key] = distances_dict[elem][next_pos][elem3]
                                    pass
                                pass
                            else:
                                for elem in hex_angle_list:
                                    for elem3 in hex_angle_list:
                                        old_key = str(h - 1) + str(elem) + old_pos + str(elem3)
                                        graph[old_key] = {}

                                        for elem4 in hex_angle_list:
                                            new_key = str(h) + str(elem3) + next_pos + str(elem4)
                                            #print "from", old_key, "to", new_key, "w:", distances_dict[elem][next_pos][elem3]
                                            graph[old_key][new_key] = distances_dict[elem3][next_pos][elem4]
                                        pass
                                    pass
                                pass
                        else:
                            graph[str(h)] = {}
                            if next_pos and next_pos != "0":

                                for elem in hex_angle_list:
                                    for elem3 in hex_angle_list:
                                        old_key = str(h - 1) + str(elem) + next_pos + str(elem3)
                                        graph[old_key] = {}
                                        graph[old_key][str(h)] = distances_dict[elem][next_pos][elem3]
                                        #graph[str(h)][old_key] = distances_dict[elem][next_pos][elem3]

                                        #print "from", old_key, "to", str(h), "w:", distances_dict[elem][next_pos][elem3]
                                    pass
                                pass
                        pass

                        counter += 1
                    pass

                    print list_hex_pos
                    print 0, str(len(list_hex_pos) -1)
                    print "graph:", graph
                    print "graph keys length:", len(graph.keys()), graph.keys()
                    print "hex_angle_list size:", len(hex_angle_list)

                    #path, distance = dijkstra(graph, '0', str(len(list_hex_pos) -1))
                    print "shortest_path", '0', str(len(list_hex_pos) -1)
                    path = shortest_path(graph, '0', str(len(list_hex_pos) -1))
                    sort_path = []
                    #for i in reversed(path):
                    for i in path:
                        sort_path.append(i)

                    print "key", key_robot, sort_path

                    angles_calculated = []

                    # Use the user defined angle variable to draw the angle of the hexagons
                    if user_selected_angle == 0:
                        sub_array = sort_path[1:-1]
                        for h in range(len(sub_array)):
                            node = sub_array[h]
                            res_re = re.findall(r'[a-z]', node)
                            if h == 0:
                                    print "res_re[0] res_re[1]", res_re[0], res_re[1]
                                    angles_calculated.append(res_re[0])
                                    angles_calculated.append(res_re[1])
                            else:
                                    print "res_re[1]", res_re[1]
                                    angles_calculated.append(res_re[1])

                        if len(angles_calculated) <= 0:
                            angles_calculated.append("a")

                        #if user_selected_algo > 1:
                        #    shuffle(angles_calculated)
                    else:
                        for i in xrange(len(hex_intersection)):
                            if user_selected_angle == 1:
                                angles_calculated.append("a")
                            elif user_selected_angle == 2:
                                angles_calculated.append("g")
                            elif user_selected_angle == 3:
                                angles_calculated.append("h")
                            elif user_selected_angle == 4:
                                angles_calculated.append("i")
                            elif user_selected_angle == 5:
                                angles_calculated.append("j")
                            elif user_selected_angle == 6:
                                angles_calculated.append("k")
                            elif user_selected_angle == 7:
                                angles_calculated.append("l")
                            elif user_selected_angle == 8:
                                angles_calculated.append("m")
                            else:
                                angles_calculated.append("n")

                    print "key", key_robot, "angles calculated:", angles_calculated
                    print "key", key_robot, "angles len:", len(angles_calculated)
                    print "key", key_robot, "points len:", len(value)

                    robot_path_angles[key_robot] = angles_calculated

                    # Create the way point list to generate the animation
                    robot_waypoints['robots'][key_robot] = []
                    robot_waypoints['robots_square'][key_robot] = []
                    path_list = robot_paths[key_robot]

                    print "path_list", path_list

                    for path_index in range(len(path_list)):
                        #print "int(path_list[path_index])", int(path_list[path_index])
                        #print "hex_intersection", hex_intersection

                        hex_poly = hex_list[hex_intersection[int(path_list[path_index]) - 1]]
                        cx = int(hex_poly.centroid.x)
                        cy = int(hex_poly.centroid.y)
                        if len(angles_calculated) <= 0:
                            hex_line_angle = 0
                        else:
                            hex_line_angle = hex_angle_dict[angles_calculated[path_index]]

                        #hex_list[6].exterior.coords[i]

                        #cover_poly = Polygon(hex_list[6].exterior.coords).buffer(0)
                        cover_center = (int(hex_list[HEXES_HIGH + 1].centroid.x) - 0, int(hex_list[HEXES_HIGH + 1].centroid.y) - 0)

                        cover_poly = Polygon(generated_leaflet_pos)
                        cover_poly = affinity.translate(cover_poly, cx - cover_center[0], cy - cover_center[1])
                        cover_poly = affinity.rotate(cover_poly, hex_line_angle)

                        sub_coords = cover_poly.exterior.coords[0:-1]
                        for h in range(len(sub_coords)):
                            x1 = sub_coords[h][0]
                            y1 = sub_coords[h][1]

                            if h + 1 < len(sub_coords):
                                x2 = sub_coords[h + 1][0]
                                y2 = sub_coords[h + 1][1]
                        pass

                        for pos in cover_poly.exterior.coords[0:-2]:
                            int_pos = (int(pos[0]), int(pos[1]))
                            robot_waypoints['robots'][key_robot].append(int_pos)

                        # Generate the 4 robots waypoints inside the hexagon
                        square_poly = Polygon(generated_square_pos)
                        square_center = (int(square_poly.centroid.x), int(square_poly.centroid.y))
                        square_poly = affinity.translate(square_poly, cx - square_center[0], cy - square_center[1])

                        sub_coords_square = square_poly.exterior.coords
                        for h in range(len(sub_coords_square)):
                            x1 = int(sub_coords_square[h][0])
                            y1 = int(sub_coords_square[h][1])
                            robot_waypoints['robots_square'][key_robot].append((x1, y1))
                            pass

                    robot_waypoints['robots'][key_robot].append(trajectory_start_pt)
                    robot_waypoints['robots_square'][key_robot].append(trajectory_start_pt)

                    print "key_robot:", key_robot, robot_waypoints['robots'][key_robot]
                    print "key_robot square:", key_robot, robot_waypoints['robots_square'][key_robot]

                pass

                robot_waypoints['start_point'] = trajectory_start_pt
                robot_waypoints['points_per_hex'] = len(generated_leaflet_pos)
                robot_waypoints['points_per_square'] = len(generated_square_pos)
                robot_waypoints['battery_autonomy'] = battery_autonomy

                robot_waypoints_json = json.dumps(robot_waypoints)
                print robot_waypoints_json

                # Calculate the clousure lines
                hexagon_centroids = {}
                different_rows = []
                for intersection_index in xrange(len(hex_intersection)):
                    print "intersection:", hex_intersection[intersection_index]
                    poly_a = hex_list[hex_intersection[intersection_index]]
                    poly_a_centroid = (int(poly_a.centroid.x), int(poly_a.centroid.y))
                    hexagon_centroids[hex_intersection[intersection_index]] = poly_a_centroid
                    if poly_a_centroid[1] not in different_rows:
                        different_rows.append(poly_a_centroid[1])
                    #poly_a = hex_list[hex_intersection[int(value[i]) - 1]]

                different_rows.sort()
                print "different_rows:", different_rows, "len:", len(different_rows)

                left_right_points = {}
                hex_by_centroid = {}
                for value in different_rows:
                    print "actual value:", value
                    left_point = 999999
                    left_point_poly = None
                    right_point = 0
                    right_point_poly = None

                    for intersection_index in xrange(len(hex_intersection)):
                        poly_a = hex_list[hex_intersection[intersection_index]]
                        poly_a_centroid = (int(poly_a.centroid.x), int(poly_a.centroid.y))

                        hex_by_centroid[poly_a_centroid] = poly_a

                        if poly_a_centroid[1] == value:
                            if poly_a_centroid[0] < left_point:
                                left_point = poly_a_centroid[0]
                                left_point_poly = poly_a

                            if poly_a_centroid[0] > right_point:
                                right_point = poly_a_centroid[0]
                                right_point_poly = poly_a

                    if left_point and right_point and left_point != right_point:
                        poly_left_point = 99999
                        poly_right_point = 0

                        for coord in left_point_poly.exterior.coords:
                            if coord[0] < poly_left_point:
                                poly_left_point = coord[0]

                        for coord in right_point_poly.exterior.coords:
                            if coord[0] > poly_right_point:
                                poly_right_point = coord[0]

                        left_right_points[value] = []
                        left_right_points[value].append((int(poly_left_point), value))
                        left_right_points[value].append((int(poly_right_point), value))
                    else:
                        poly_alone = None
                        if left_point_poly:
                            poly_alone = left_point_poly
                        else:
                            poly_alone = right_point_poly

                        poly_left_point = 99999
                        poly_right_point = 0

                        for coord in poly_alone.exterior.coords:
                            if coord[0] < poly_left_point:
                                poly_left_point = coord[0]

                            if coord[0] > poly_right_point:
                                poly_right_point = coord[0]

                        left_right_points[value] = []
                        left_right_points[value].append((int(poly_left_point), value))
                        left_right_points[value].append((int(poly_right_point), value))
                    pass

                print "left_right_points:", left_right_points
                external_points = []
                for key in sorted(left_right_points.keys()):
                    print "key:", key
                    for elem in left_right_points[key]:
                        external_points.append(elem)

                robot_waypoints['external_points'] = external_points

                #begin_num = ord('a')
                #genes = [chr(i) for i in range(0, number_of_robots)]
                #genes = genes + [i for i in range(num_of_edges)]

                print "hex_positions", hex_positions
                print "robot_paths_array", robot_paths_array

                distances_hex_positions = {}

                for key, value in hex_positions.items():
                    distances_hex_positions[key] = {}
                    for key2, value2 in hex_positions.items():
                        if key == key2:
                            distances_hex_positions[key][key2] = 0
                            continue
                        d, tetha = points_to_vector(value, value2)
                        distances_hex_positions[key][key2] = d

                fitness_value = evaluate(robot_paths_array, hex_positions, distances_hex_positions, trajectory_start_pt, ga_object['adjacency'])

                #print "Start", start_time
                #print "End", end_time
                print "**** Time to execute the algorithm:", calculated_time.total_seconds()

                with open("/tmp/result_time_and_fitness.txt", "a") as my_file:
                    msg = "Algo:" + str(user_selected_algo_description[user_selected_algo]) + " Nrobots:" + str(number_of_robots) + \
                          " Fitness:" + str(fitness_value) + " Time:" + str(calculated_time.total_seconds())
                    my_file.write(msg + "\n")

                # End sim
                continue_simulation = False

                #with open('data.txt', 'w') as outfile:
                #    json.dump(data, outfile)

        # Test Ollero implementation

        if continue_simulation_zigzag:

            print "hex_intersection len", len(hex_intersection)

            # poly_list = []
            # for index in hex_intersection:
            #     hex_poly = hex_list[index]
            #     poly_list.append(hex_poly)
            #
            # # Point list of all H'
            # ollero_point_list = []
            # for polygon_o in poly_list:
            #     for coord in polygon_o.exterior.coords:
            #         ollero_point_list.append((int(coord[0]), int(coord[1])))
            #
            # print "ollero_point_list", ollero_point_list
            #
            # convex_hull_array = convex_hull(ollero_point_list)
            # convex_hull_array.append(convex_hull_array[0])
            #
            # convex_hull_array.reverse()

            convex_hull_array = point_list
            convex_hull_array.append(convex_hull_array[0])
            convex_hull_array.reverse()

            ollero_json = {}
            ollero_json['point_list'] = convex_hull_array
            ollero_json['k_number'] = number_of_robots
            ollero_json['start_point'] = trajectory_start_pt

            ollero_json_dump = json.dumps(ollero_json)

            print "ollero_json_dump:", ollero_json_dump

            p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/hex_manual_astar/ollero_implementation.py", ollero_json_dump], stdin=None, stdout=None, stderr=None)


            # cascaded_poly_union = ops.unary_union(poly_list)
            # if cascaded_poly_union.geom_type == "MultiPolygon":
            #     poly_union = None
            #     print "Poly union is multipolygon... cannot create points, please use other figure."
            # else:
            #     poly_union = Polygon(cascaded_poly_union.exterior.coords)
            #     json_str = generate_json_string_from_list(cascaded_poly_union.exterior.coords)
            #     #json_str = "'"+str(json_str)+"'"
            #     print "json_str", json_str
            #     p = subprocess.Popen(["python", "/Users/h3ct0r/Desktop/polygon_cover_python/coverage/CoveragePathSim.py", json_str], stdout=subprocess.PIPE)
            #     out, err = p.communicate()
            #     outSplit = out.split()
            #     print "RESULT LEAFLET", outSplit
            #
            #     out_list = []
            #     jump = True
            #     for i in xrange(len(outSplit)):
            #         jump = not jump
            #         if not jump:
            #             out_list.append( (int(outSplit[i]), int(outSplit[i+1])) )
            #
            #     out_list.append(trajectory_start_pt)
            #
            #     # Apply K-splittour
            #     cost_list = []
            #     for i in xrange(len(out_list) - 1):
            #         pos1 = out_list[i]
            #         pos2 = out_list[i + 1]
            #         d, theta = points_to_vector(pos1, pos2)
            #         cost_list.append(int(d))
            #
            #     L = sum(cost_list)              # L = the sum of all costs
            #     c_max = max(cost_list)          # C_max = the max cost of the complete tour
            #
            #     j = 1
            #     k = number_of_robots
            #     base = 0
            #     cost = 0
            #     total_routes = []
            #
            #     # Initial cost is the cost of 0 to 1
            #     cost = cost_list[0]
            #     print "initial cost:", cost
            #
            #     for i in xrange(len(out_list) - 1):
            #
            #         actual_pos = out_list[i]
            #         next_pos = out_list[i + 1]
            #
            #         if j >= k:
            #             break
            #
            #         threshold = (j/float(k)) * (L - (2 * c_max)) + c_max
            #
            #         d, theta = points_to_vector(actual_pos, next_pos)
            #
            #         cost += int(d)
            #         print "cost from actual to next:", int(d)
            #
            #         print "threshold:", threshold
            #         print "cost:", cost
            #
            #         if cost > threshold:
            #             j += 1
            #             d, theta = points_to_vector(out_list[0], next_pos)
            #             cost = int(d)
            #
            #             tour = out_list[base:i+1]
            #             tour.append(trajectory_start_pt)
            #
            #             base = i + 1
            #             total_routes.append(tour)
            #
            #     if base < len(out_list) - 1:
            #         tour = out_list[base:]
            #         total_routes.append(tour)
            #
            #     print "total_routes:", total_routes
            #
            #     begin_num = ord('a')
            #     letters_generated = [chr(i) for i in range(begin_num, begin_num + number_of_robots)]
            #
            #
            #     new_robot_waypoints = {}
            #     new_robot_waypoints['start_point'] = trajectory_start_pt
            #     new_robot_waypoints['robots'] = {}
            #
            #     for route in total_routes:
            #         letter = letters_generated.pop(0)
            #         new_robot_waypoints['robots'][letter] = route
            #
            #     #new_robot_waypoints = {}
            #     #new_robot_waypoints['start_point'] = trajectory_start_pt
            #     #new_robot_waypoints['robots'] = {}
            #     #new_robot_waypoints['robots']['a'] = out_list
            #
            #     new_robot_waypoints_json = json.dumps(new_robot_waypoints)
            #
            #     p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/hex_manual_astar/robot_movement.py", new_robot_waypoints_json], stdout=subprocess.PIPE)
            #     out, err = p.communicate()
            #     outSplit = out.split()
            #     pass

            continue_simulation_zigzag = False
            pass
        # End ollero implementation

        # Start drawing ---------------------------------------------------------------


        # Draw the color of the hexagon
        for key, value in robot_paths.items():
            color = robot_colors[key]
            counter = 1
            for index in value:
                hex_poly = hex_list[hex_intersection[int(index) - 1]]
                coords = hex_poly.exterior.coords

                pygame.draw.polygon(screen, color, coords)

                #label = monoFont.render(str(counter), 2, (0, 0, 0))
                #screen.blit(label, (int(hex_poly.centroid.x), int(hex_poly.centroid.y)))
                #counter += 1
            pass

        """
        # Draw the centrooids
        for index in hex_intersection:
            hex_poly = hex_list[index]
            coords = hex_poly.exterior.coords

            #pygame.draw.polygon(screen, GREEN, coords)

            pygame.draw.circle(screen, GREEN, (int(hex_poly.centroid.x), int(hex_poly.centroid.y)), 3)
        """

        # Draw the white polygons
        for hex_poly in hex_list:
            coords = hex_poly.exterior.coords
            pygame.draw.polygon(screen, WHITE, coords, 1)

        # Draw the white points to denote the bigger area
        for border in point_list:
            pygame.draw.circle(screen, WHITE, border, 2)

        # Draw the white lines between the white dots
        if len(robot_path_angles.keys()) == 0:
            for i in xrange(len(point_list)):
                x1 = point_list[i][0]
                y1 = point_list[i][1]

                if(i+1 < len(point_list)):
                  x2 = point_list[i+1][0]
                  y2 = point_list[i+1][1]

                  pygame.draw.line(screen, WHITE, [x2, y2], [x1, y1])

        # Draw adjacency list
        '''
        for key, value in adjacency_dict.items():
            poly_a = hex_list[key]
            poly_a_center = (int(poly_a.centroid.x), int(poly_a.centroid.y))

            for index in value:
                poly_b = hex_list[index]
                poly_b_center = (int(poly_b.centroid.x), int(poly_b.centroid.y))

                pygame.draw.line(screen, RED, poly_a_center, poly_b_center)

                #print poly_a_center, poly_b_center
        '''

        # print "robot_path_angles", robot_path_angles
        # print "hex_angle_dict", hex_angle_dict

        # Draw the paths inside the hexagon
        for key, value in robot_path_angles.items():
            path_list = robot_paths[key]
            # print "value", value
            # print "path_list", path_list

            total_distance = 0

            for path_index in range(len(path_list)):
                # print path_index
                # hex_poly = hex_list[hex_intersection[int(path_list[path_index])]]
                hex_poly = hex_list[hex_intersection[int(path_list[path_index]) - 1]]
                cx = int(hex_poly.centroid.x)
                cy = int(hex_poly.centroid.y)
                hex_line_angle = hex_angle_dict[value[path_index]]

                #cover_poly = Polygon(generated_leaflet_pos).buffer(0)
                cover_center = (int(hex_list[HEXES_HIGH + 1].centroid.x) - 0, int(hex_list[HEXES_HIGH + 1].centroid.y) - 0)
                #cover_center = (int(cover_poly.centroid.x) - 0, int(cover_poly.centroid.y) - 20)

                cover_poly = Polygon(generated_leaflet_pos)
                cover_poly = affinity.translate(cover_poly, cx - cover_center[0], cy - cover_center[1])
                cover_poly = affinity.rotate(cover_poly, hex_line_angle)

                sub_coords = cover_poly.exterior.coords[0:-1]
                for h in range(len(sub_coords)):
                    x1 = sub_coords[h][0]
                    y1 = sub_coords[h][1]

                    if h + 1 < len(sub_coords):
                        x2 = sub_coords[h + 1][0]
                        y2 = sub_coords[h + 1][1]
                        pygame.draw.line(screen, WHITE, (x1, y1), (x2, y2), 1)

                        d, ang = points_to_vector((x1, y1), (x2, y2))
                        total_distance += d
                    pass

                # Draw the 4 robots inside the hexagon
                square_poly = Polygon(generated_square_pos)
                square_center = (int(square_poly.centroid.x), int(square_poly.centroid.y))
                square_poly = affinity.translate(square_poly, cx - square_center[0], cy - square_center[1])

                sub_coords_square = square_poly.exterior.coords
                for h in range(len(sub_coords_square)):
                    x1 = int(sub_coords_square[h][0])
                    y1 = int(sub_coords_square[h][1])
                    pygame.draw.circle(screen, BLACK, (x1, y1), 5)
                    pygame.draw.circle(screen, LIGHT_BLUE, (x1, y1), 3)
                    pass

                label = monoFont.render(str(path_index + 1) + "S", 1, BLUE)
                screen.blit(label, cover_poly.exterior.coords[0])

                label = monoFont.render(str(path_index + 1) + "E", 1, BLUE)
                screen.blit(label, cover_poly.exterior.coords[-2])

                #print "path index", int(path_index)

                # TODO: Optimize this!

                if int(path_index) + 1 < len(path_list):
                    next_hex_line_angle = hex_angle_dict[value[int(path_index) + 1]]
                    #hex_poly2 = hex_list[hex_intersection[int(path_list[int(path_index) + 1])]]
                    hex_poly2 = hex_list[hex_intersection[int(path_list[int(path_index) + 1]) - 1]]
                    cx2 = int(hex_poly2.centroid.x)
                    cy2 = int(hex_poly2.centroid.y)

                    cover_poly2 = Polygon(generated_leaflet_pos)
                    cover_poly2 = affinity.translate(cover_poly2, cx2 - cover_center[0], cy2 - cover_center[1])
                    cover_poly2 = affinity.rotate(cover_poly2, next_hex_line_angle)

                    pygame.draw.line(screen, RED, cover_poly.exterior.coords[-2], cover_poly2.exterior.coords[0])

                    d, ang = points_to_vector(cover_poly.exterior.coords[-2], cover_poly2.exterior.coords[0])
                    total_distance += d

                    if int(path_index) == 0:
                        next_hex_line_angle = hex_angle_dict[value[int(path_index)]]
                        #hex_poly2 = hex_list[hex_intersection[int(path_list[int(path_index)])]]
                        hex_poly2 = hex_list[hex_intersection[int(path_list[int(path_index)]) - 1]]
                        cx2 = int(hex_poly2.centroid.x)
                        cy2 = int(hex_poly2.centroid.y)

                        cover_poly2 = Polygon(generated_leaflet_pos)
                        cover_poly2 = affinity.translate(cover_poly2, cx2 - cover_center[0], cy2 - cover_center[1])
                        cover_poly2 = affinity.rotate(cover_poly2, next_hex_line_angle)

                        pygame.draw.line(screen, GREEN, cover_poly.exterior.coords[0], trajectory_start_pt)

                        d, ang = points_to_vector(cover_poly.exterior.coords[0], trajectory_start_pt)
                        total_distance += d
                else:
                    next_hex_line_angle = hex_angle_dict[value[path_index]]
                    #hex_poly2 = hex_list[hex_intersection[int(path_list[int(path_index)])]]
                    hex_poly2 = hex_list[hex_intersection[int(path_list[int(path_index)]) - 1]]
                    cx2 = int(hex_poly2.centroid.x)
                    cy2 = int(hex_poly2.centroid.y)

                    cover_poly2 = Polygon(generated_leaflet_pos)
                    cover_poly2 = affinity.translate(cover_poly2, cx2 - cover_center[0], cy2 - cover_center[1])
                    cover_poly2 = affinity.rotate(cover_poly2, next_hex_line_angle)

                    pygame.draw.line(screen, YELLOW, cover_poly.exterior.coords[-2], trajectory_start_pt)

                    d, ang = points_to_vector(cover_poly.exterior.coords[-2], trajectory_start_pt)
                    total_distance += d
                    pass

            print "Total distance robot:", key, "distance:", total_distance
            pass

        if poly_union and len(poly_union.exterior.coords) > 2:
            pygame.draw.polygon(screen, GREEN, poly_union.exterior.coords, 5)

        # Draw the number of the hexagon
        for key, value in robot_paths.items():
            color = robot_colors[key]
            counter = 1

            for index in value:
                hex_poly = hex_list[hex_intersection[int(index) - 1]]

                label = monoFont20.render(str(counter), 2, (0, 0, 0))
                if counter > 9:
                    screen.blit(label, (int(hex_poly.centroid.x) - 10, int(hex_poly.centroid.y) - 5))
                else:
                    screen.blit(label, (int(hex_poly.centroid.x) - 5, int(hex_poly.centroid.y) - 5))
                counter += 1
            pass

        # Draw starting point
        pygame.draw.circle(screen, YELLOW, trajectory_start_pt, 4)

        pygame.draw.rect(screen, WHITE, (0, 560, 620, 600))

        # 95 PIX 10 mts
        #label = monoFont.render("MINA DE CAPANEMA ", 1, BLACK)
        label = monoFont.render("CAMPO FUTEBOL UFMG ", 1, BLACK)
        screen.blit(label, (20, 580))

        #pygame.draw.line(screen, BLACK, (170, 570), (265, 570), 4)
        #label = monoFont.render("20 mts", 1, BLACK)

        pygame.draw.line(screen, BLACK, (170, 570), (220, 570), 4)
        label = monoFont.render("6.5 mts", 1, BLACK)
        screen.blit(label, (170, 580))

        #label = monoFont.render("Approx GPS  644747.62, 767052.53", 1, BLACK)
        #screen.blit(label, (350, 580))

        pygame.draw.rect(screen, BLACK, (100, 0, 30, 40))
        label = monoFont20.render(str(lines_width), 10, WHITE)
        screen.blit(label, (102, 10))

        pygame.draw.rect(screen, BLACK, (230, 0, 50, 40))
        label = monoFont20.render(str(battery_autonomy), 10, WHITE)
        screen.blit(label, (230, 10))

        pygame.draw.rect(screen, BLACK, (380, 0, 30, 40))
        label = monoFont20.render(str(RADIUS), 10, WHITE)
        screen.blit(label, (382, 10))

        pygame.draw.rect(screen, BLACK, (510, 0, 30, 40))
        label = monoFont20.render(str(number_of_robots), 10, WHITE)
        screen.blit(label, (512, 10))

        pygame.draw.rect(screen, BLACK, (590, 0, 30, 40))
        label = monoFont20.render(str(user_selected_angle), 10, WHITE)
        screen.blit(label, (595, 10))

        pygame.draw.rect(screen, BLACK, (670, 0, 110, 40))
        label = monoFont20.render(str(user_selected_algo_description[user_selected_algo]), 10, WHITE)
        screen.blit(label, (675, 10))

        redraw = False

    clock.tick(30)
    pygame.display.flip()
    pygame.display.set_caption('FPS: ' + str(clock.get_fps()))

pygame.quit()