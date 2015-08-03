__author__ = 'h3ct0r'

from math import sin, cos, pi, sqrt, atan2, degrees
import random
from shapely.geometry import *
from shapely import affinity
import pygame
import re
import sys
from shapely.geometry import *
from shapely import affinity
import json
import Image
import math
import json
import subprocess
import sys

sys.setrecursionlimit(1500)

BLACK = (0,   0,   0)
WHITE = (255, 255, 255)
GREEN = (0, 255,   0)
RED = (255,   0,   0)
BLUE = (0,  60,   255)
YELLOW = (255, 255,   0)

IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600

pygame.init()
mono_font = pygame.font.SysFont("monospace", 12)
clock = pygame.time.Clock()

screen = pygame.display.set_mode( (IMAGE_WIDTH, IMAGE_HEIGHT) )

is_loop_active = True

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

# def area(p):
#     return 0.5 * abs(sum(x0*y1 - x1*y0
#                          for ((x0, y0), (x1, y1)) in segments(p)))
#
# def segments(p):
#     return zip(p, p[1:] + [p[0]])

def generate_voronoi_diagram_with_border(width, height, pointList, borderList):
    image = Image.new("RGB", (width, height))
    getpixel = image.getpixel
    putpixel = image.putpixel
    imgx, imgy = image.size

    print "Point list size:", len(pointList)
    if len(pointList) <= 0:
        print "Not enough points"
        return

    coords = []
    for border in borderList:
        coords.append( (border[0], border[1]) )

    print coords

    border_poly = Polygon(coords)

    colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for i in xrange(len(pointList))]

    # polygon_inner_points = []
    # for y in range(imgy):
    #     for x in range(imgx):
    #         point = Point(x, y)
    #         if(border_poly.intersects(point) or border_poly.touches(point)or border_poly.within(point) or border_poly.crosses(point)):
    #             polygon_inner_points.append((x, y))
    #
    # for point in polygon_inner_points:
    #     x = point[0]
    #     y = point[1]

    # leftmost_point = -1
    # rightmost_point = -1
    # upmost_point = -1
    # downmost_point = -1
    #
    # for point in borderList:
    #     x = point[0]
    #     y = point[1]
    #     if x < leftmost_point or leftmost_point == -1:
    #         leftmost_point = x
    #     if x > rightmost_point or rightmost_point == -1:
    #         rightmost_point = x
    #     if y > upmost_point or upmost_point == -1:
    #         upmost_point = y
    #     if y < downmost_point or downmost_point == -1:
    #         downmost_point = y

    temp = map(sorted, zip(*borderList))
    min_x, max_x, min_y, max_y = temp[0][0], temp[0][-1], temp[1][0], temp[1][-1]

    print "min_x: ", min_x, " max_x: ", max_x, " min_y: ", min_y, "max_y: ", max_y

    borderDict = {}

    for y in xrange(min_y, max_y):
        for x in xrange(min_x, max_x):
            #if (x < leftmost_point or x > rightmost_point) and (y > upmost_point or y < downmost_point):
            #    continue
            dmin = math.hypot(imgx-1, imgy-1)
            j = -1
            for i in xrange(len(pointList)):
                border = pointList[i]
                d = math.hypot(border[0]-x, border[1]-y)
                if d < dmin:
                    dmin = d
                    j = i

            point = Point(x, y)
            if(border_poly.intersects(point) or border_poly.touches(point)or border_poly.within(point) or border_poly.crosses(point)):
                putpixel((x, y), colors[j])

    print "Phase 1"

    # Find the intersection of 3 colors
    for y in range(imgy):
        for x in range(imgx):
            original_pixel = getpixel((x, y))

            # We suposse no color is going to be black!
            if original_pixel == (0,0,0):
                continue

            #print (x,y)
            colorsFound = []
            for i in range(-1, 2):
                for j in range(-1, 2):
                    if((0 <= x + i <= width) and (0 <= y + j <= height)):
                        pixel = getpixel((x + i, y + j))

                        if pixel not in colorsFound:
                            colorsFound.append(pixel)

            if len(colorsFound) >= 3:
                if original_pixel not in borderDict:
                    borderDict[original_pixel] = []

                borderDict[original_pixel].append( (x, y) )
    print "Phase 2"

    # Add the borders
    for i in xrange(len(borderList)):
        dmin = math.hypot(imgx-1, imgy-1)
        border = borderList[i]
        minpos  = 0
        for j in xrange(len(pointList)):
            point = pointList[j]
            d = math.hypot(border[0]-point[0], border[1]-point[1])
            if d < dmin:
                dmin = d
                minpos = j

        if colors[minpos] not in borderDict:
            borderDict[colors[minpos]] = []

        borderDict[colors[minpos]].append( (border[0], border[1]) )
    print "Phase 3"

    image.save("LastGeneratedVoronoiDiagram.png", "PNG")
    #image.show()
    return borderDict


json_string = sys.argv[1]
json_data = json.loads(json_string)
start_point = json_data['start_point']
NUMBER_ROBOT = json_data['k_number']
pos_array = json_data['point_list']

convex_hull_array = convex_hull(pos_array)
convex_hull_array.append(convex_hull_array[0])

convex_hull_array.reverse()

#S = [random.choice(convex_hull_array) for i in xrange(3)]
S = []
lines_width = 10
if 'lines_width' in json_data:
    lines_width = json_data['lines_width']

amostral_space_file = json_data['amostral_space_file']

S_segment = int(len(convex_hull_array) / NUMBER_ROBOT)
for i in xrange(NUMBER_ROBOT):
    S.append(convex_hull_array[(i - 1) * S_segment])

voronoi = generate_voronoi_diagram_with_border(800, 600, S, convex_hull_array)
voronoi_polygons = []
voronoi_lawnmower = []

for key in voronoi.keys():
    convex_points = convex_hull(voronoi[key])
    voronoi_polygons.append(Polygon(convex_points))

    coverage_path_points = "["
    for i in xrange(len(convex_points)):
        point = convex_points[i]
        coverage_path_points += "[" + str(int(point[0])) + "," + str(int(point[1])) + "]"
        if i < len(convex_points) - 1:
            coverage_path_points += ","
    coverage_path_points += "]"

    print "coverage_path_points", coverage_path_points

    p = subprocess.Popen(["python",
                          "/Users/h3ct0r/PycharmProjects/hex_route_msc/coverage_path_verticalseg_rotate_vert_lines.py",
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

    voronoi_lawnmower.append(generated_leaflet_pos)

#print "area:", area(convex_hull_array)

print "shapely area:", Polygon(convex_hull_array).area
print "S:", S
#print "voronoi:", voronoi

robot_waypoints = {}
robot_waypoints['robots'] = {}

for robot_key in xrange(NUMBER_ROBOT):
    robot_waypoints['robots'][str(robot_key)] = voronoi_lawnmower[robot_key]

robot_waypoints['start_point'] = start_point
robot_waypoints['battery_autonomy'] = 999999
robot_waypoints['amostral_space_file'] = amostral_space_file
robot_waypoints['points_per_hex'] = 99 # hardcoded
robot_waypoints['points_per_square'] = 99 # hardcoded

robot_waypoints_json = json.dumps(robot_waypoints)

while is_loop_active:

    screen.fill(BLACK)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            is_loop_active = False

        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_SPACE):

            p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/hex_route_msc/robot_movement_with_sampling_gaussian.py", robot_waypoints_json], stdin=None, stdout=None, stderr=None)

            # for i in xrange(10):
            #     p = subprocess.Popen(["python", "/Users/h3ct0r/PycharmProjects/hex_manual_astar/robot_movement.py", robot_waypoints_json], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            #     output, error = p.communicate()
            #     status = p.wait()


    for polygon in voronoi_polygons:
        points_polygon = polygon.exterior.coords
        for i in xrange(len(points_polygon)):
            pos = points_polygon[i]

            label = mono_font.render(str(i + 1), 1, BLUE)
            screen.blit(label, (pos[0] + 10, pos[1]))

            if i+1 < len(points_polygon):
                next_pos = points_polygon[i + 1]
                pygame.draw.line(screen, WHITE, next_pos, pos)

    for polygon in voronoi_lawnmower:
        for i in xrange(len(polygon)):
            pos = polygon[i]

            #label = mono_font.render(str(i + 1), 1, BLUE)
            #screen.blit(label, (pos[0] + 10, pos[1]))

            if i+1 < len(polygon):
                next_pos = polygon[i + 1]
                pygame.draw.line(screen, YELLOW, next_pos, pos)

    # for i in xrange(len(convex_hull_array)):
    #     pos = convex_hull_array[i]
    #
    #     label = mono_font.render(str(i + 1), 1, BLUE)
    #     screen.blit(label, (pos[0] + 10, pos[1]))
    #
    #     if i+1 < len(convex_hull_array):
    #         next_pos = convex_hull_array[i + 1]
    #         pygame.draw.line(screen, WHITE, next_pos, pos)

    for i in xrange(len(S)):
            pos = S[i]

            label = mono_font.render(str(i + 1), 1, RED)
            screen.blit(label, (pos[0], pos[1] + 5))
            pygame.draw.circle(screen, RED, pos, 4)

    # for i in xrange(len(pos_array)):
    #     pos_index = pos_array[i]
    #     x1 = adjdata['distances'][str(pos_index)][0]
    #     y1 = adjdata['distances'][str(pos_index)][1]
    #
    #     #print (x1, y1)
    #
    #     if i+1 < len(pos_array):
    #         next_index = pos_array[i + 1]
    #         x2 = adjdata['distances'][str(next_index)][0]
    #         y2 = adjdata['distances'][str(next_index)][1]
    #
    #         #print (x2, y2)
    #
    #         pygame.draw.line(screen, WHITE, [x2, y2], [x1, y1])
    #
    #     label = mono_font.render(str(i + 1), 1, BLUE)
    #     screen.blit(label, (x1 + 10, y1))
    #
    #     label = mono_font.render(str(i), 1, WHITE)
    #     screen.blit(label, (adjdata['distances'][pos_index][0] + 20, adjdata['distances'][pos_index][1]))
    #
    #     pygame.draw.circle(screen, WHITE, [x1, y1], 2)

    clock.tick(30)
    pygame.display.flip()
    pygame.display.set_caption('FPS: ' + str(clock.get_fps()))

    pygame.draw.rect(screen, BLACK, [0, IMAGE_WIDTH - 25, IMAGE_HEIGHT, 25])
    label = mono_font.render(" Press space to run simulation", 1, (255, 255, 0))
    screen.blit(label, (10, IMAGE_WIDTH - 25))

    pass