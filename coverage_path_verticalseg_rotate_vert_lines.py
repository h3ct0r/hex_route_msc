import pygame
import sys
import math
import json
import subprocess
import shapely
from shapely.geometry import *
from shapely import affinity
import heapq
import time

import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from geometry_helper import *

def line(x0, y0, x1, y1):
    "Bresenham's line algorithm"
    point_list = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            point_list.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            point_list.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    point_list.append((x, y))
    return point_list

# Define some colors
DEEP_BLUE = (   0,   0,   50)
BLACK     = (   0,   0,   0)
WHITE     = ( 255, 255, 255)
GREEN     = (   0, 255,   0)
RED       = ( 255,   0,   0)
BLUE      = (   0,  60,   255)
YELLOW    = ( 255, 255,   0)

# Geosoft : program to create waypoints
# Restriction from south to north
# CPE tecnologia, VANT courses
#   Vant stmart one aibotix

pygame.init()
monoFont = pygame.font.SysFont("monospace", 15)

sizeX = 800
sizeY = 600
size = [sizeX, sizeY]
screen = pygame.display.set_mode(size)

clickedPos = []
"""
# Hex small
clickedPos = [

    ( 70 , 69 ),
( 60 , 86 ),
( 40 , 86 ),
( 30 , 69 ),
( 39 , 51 ),
( 59 , 51 ),
( 70 , 69 )

]
"""

"""
# Hex grande
clickedPos = [
    ( 262 , 259 ),
    ( 225 , 324 ),
    ( 150 , 324 ),
    ( 112 , 259 ),
    ( 149 , 194 ),
    ( 224 , 194 ),
    ( 262 , 259 )
]
"""

"""
#Big hex
clickedPos = [
    ( 422 , 112 ),
( 390 , 168 ),
( 325 , 168 ),
( 292 , 112 ),
( 325 , 56 ),
( 389 , 56 ),
( 422 , 112 )
]
"""

"""
clickedPos = [

    (100, 100),
    (200, 100),
    (100, 200),
    (200, 200)

]
"""

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

lineWidth = 10

#"""
pointsJson = sys.argv[1]
clickedPos = json.loads(pointsJson)
clickedPos.append(clickedPos[0])

if len(sys.argv) == 3:
    lineWidth = int(sys.argv[2])
#"""

min_left = sizeX
for pos in clickedPos:
    left = pos[0]
    if left < min_left:
        min_left = left

print "min_left:", min_left

done = False
drawLines = False
lineAngle = -1

#simulationState = 'GENERATE_POLYGON'
simulationState = 'CALCULATE_BEST_ANGLE'
#simulationState = 'SHOW_BEST_POLYGON'

lineAngle = 0

#numberOfLines = int(math.ceil(abs(sizeX - min_left)/lineWidth))
numberOfLines = int(math.ceil(abs(sizeX * 1.5)/lineWidth))

bestSize = 0
bestAngle = 0
bestPositions = []
bestPositionsRotated = []

cx = 0
cy = 0

# Used to delimite the calculations of the lines
min_x = sizeX
max_x = 0

clock = pygame.time.Clock()

tsp_coords = []
square_coords = []
last_position = 0

big_polygon_shape = Polygon(clickedPos)

basePath = "/Users/h3ct0r/Sites/"

# -------- Main Program Loop -----------
while done == False:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True

    # Set the screen background
    screen.fill(BLACK)
    color = WHITE

    if(simulationState == 'GENERATE_POLYGON'):
      # Draw all previous selected points
      if(len(clickedPos) > 0):
        for i in xrange(len(clickedPos)):
          pygame.draw.rect(screen, color, [clickedPos[i][0], clickedPos[i][1], 5, 5])

          if(i > 0 and i < len(clickedPos)):
            pygame.draw.line(screen, color, [clickedPos[i-1][0], clickedPos[i-1][1]], [clickedPos[i][0], clickedPos[i][1]])

        # Draw current mouse pos
        lastClicked = clickedPos[-1]
        actualMousePos = pygame.mouse.get_pos()
        pygame.draw.line(screen, color, lastClicked, actualMousePos)

    elif(simulationState == 'CALCULATE_BEST_ANGLE'):

        lineAngle += 10
        if lineAngle > 360:
            lineAngle = 0
            simulationState = 'SHOW_BEST_POLYGON'

        local_intersections = 0
        local_positions = []
        for i in xrange(0 - numberOfLines/2, numberOfLines):
            x_pos_line = i * lineWidth
            line_poly = LineString([(x_pos_line, -600), (x_pos_line, sizeY + 600)])
            line_poly = affinity.rotate(line_poly, lineAngle, origin=(sizeX/2, sizeY/2))

            line_coords = list(line_poly.coords)

            points_in_line = line(int(line_coords[0][0]), int(line_coords[0][1]), int(line_coords[1][0]), int(line_coords[1][1]))

            if big_polygon_shape.intersects(line_poly):
                insersec_points = big_polygon_shape.intersection(line_poly)
                if isinstance(insersec_points, MultiLineString):
                    insersec_points = insersec_points.geoms[0]

                if len(insersec_points.coords) > 1:
                    a1 = Point(insersec_points.coords[0])
                    a2 = Point(insersec_points.coords[1])

                    #print "a1:", a1
                    #print "a2:", a2

                    local_positions.append((int(a1.x), int(a1.y)))
                    local_positions.append((int(a2.x), int(a2.y)))

                    pygame.draw.line(screen, RED, (int(a1.x), int(a1.y)), (int(a2.x), int(a2.y)))
                    local_intersections += 1

        if local_intersections < bestSize or bestSize == 0:
            bestAngle = lineAngle
            bestSize = local_intersections
            bestPositionsRotated = local_positions

    elif(simulationState == 'GENERATE_LIST_OF_POINTS'):

      # Rotate the polygon
      for i in xrange(len(bestPositions)):
        point = bestPositions[i]
        x1,y1 = rotate2d(bestAngle * -1, point, (cx,cy))
        bestPositionsRotated.append((x1,y1))

      #print "bestPositionsRotated", bestPositionsRotated
      simulationState = 'SHOW_BEST_POLYGON'

    elif(simulationState == 'SHOW_BEST_POLYGON'):
      # Draw original polygon
      if(len(clickedPos) > 0):
        for i in xrange(len(clickedPos)):
          pygame.draw.rect(screen, color, [clickedPos[i][0], clickedPos[i][1], 5, 5])
          if(i > 0 and i < len(clickedPos)):
            pygame.draw.line(screen, color, [clickedPos[i-1][0], clickedPos[i-1][1]], [clickedPos[i][0], clickedPos[i][1]])

      # Draw generated points
      #for i in xrange(len(bestPositions)):
      #  x1,y1 = bestPositions[i]
      #  pygame.draw.line(screen, BLUE, [x1,y1], [x1,y1])
      #  if(i > 0 and i < len(bestPositions)):
      #    x2,y2 = bestPositions[i-1]
      #    pygame.draw.line(screen, BLUE, [x2,y2], [x1,y1])

      # Draw the rotated polygon
      for i in xrange(len(bestPositionsRotated)):
        x1, y1 = bestPositionsRotated[i]

        # Show the point number on the screen
        labelNumber = monoFont.render(str(i+1), 1, (255,255,0))
        screen.blit(labelNumber, (int(x1), int(y1)-15))

        pygame.draw.rect(screen, BLUE, [int(x1), int(y1), 5, 5])
        if(i > 0 and i < len(bestPositionsRotated)):
          x2,y2 = bestPositionsRotated[i-1]
          pygame.draw.line(screen, BLUE, [x2,y2], [x1,y1])

      simulationState = 'CALCULATE_TSP_SIMPLE'

    elif(simulationState == 'CALCULATE_TSP_SIMPLE'):
        weightMatrix = {}
        isEvenCount = 1
        isNotEvenCount = 0
        for i in xrange(len( bestPositionsRotated )):
          weightMatrix[i] = {}

          isEven = False
          if(i % 2 == 0):
            isEven = True
            if(isEvenCount == 0):
              isEvenCount += 1
            else:
              isEvenCount = 0
          else:
            if(isNotEvenCount == 0):
              isNotEvenCount += 1
            else:
              isNotEvenCount = 0

          for y in xrange(len( bestPositionsRotated )):
            #print "i", i, "y", y
            if(y == i):
              weightMatrix[i][y] = 0
              pass
            elif(isEven):
              if(isEvenCount == 0):
                if(y == i+1 or (y == i-2 and i != 0)):
                  weightMatrix[i][y] = 1
                else:
                  weightMatrix[i][y] = 999
              else:
                if(y == i+2 or (y == i+1 and i != 0)):
                  weightMatrix[i][y] = 1
                else:
                  weightMatrix[i][y] = 999
            else:
              if(isNotEvenCount == 0):
                if(y == i-1 or y == i-2):
                  weightMatrix[i][y] = 1
                else:
                  weightMatrix[i][y] = 999
              else:
                if(y == i+2 or y == i-1):
                  weightMatrix[i][y] = 1
                else:
                  weightMatrix[i][y] = 999

        print "weightMatrix", weightMatrix

        path = shortest_path(weightMatrix, 0, len(bestPositionsRotated) - 2)
        if len(path) != len( bestPositionsRotated):
          path = shortest_path(weightMatrix, 0, len(bestPositionsRotated) - 1)

        print "path", path, len(path)

        tsp_coords = []
        for index in path:
            tsp_coords.append((int(bestPositionsRotated[index][0]), int(bestPositionsRotated[index][1])) )
        pass

        print "tsp_coords", tsp_coords

        simulationState = 'SHOW_TSP'

    elif(simulationState == 'CALCULATE_TSP'):
        # Generate the matrix of the values
        # And write the contents of the TSPLIB file
        weightMatrix = []
        isEvenCount = 1
        isNotEvenCount = 0
        for i in xrange(len( bestPositionsRotated )):
          weightMatrix.append([])

          isEven = False
          if(i % 2 == 0):
            isEven = True
            if(isEvenCount == 0):
              isEvenCount += 1
            else:
              isEvenCount = 0
          else:
            if(isNotEvenCount == 0):
              isNotEvenCount += 1
            else:
              isNotEvenCount = 0

          for y in xrange(len( bestPositionsRotated )):
            if(y == i):
              weightMatrix[i].append(0)
            elif(isEven):
              if(isEvenCount == 0):
                if(y == i+1 or (y == i-2 and i != 0)):
                  weightMatrix[i].append(1)
                else:
                  weightMatrix[i].append(999)
              else:
                if(y == i+2 or (y == i+1 and i != 0)):
                  weightMatrix[i].append(1)
                else:
                  weightMatrix[i].append(999)
            else:
              if(isNotEvenCount == 0):
                if(y == i-1 or y == i-2):
                  weightMatrix[i].append(1)
                else:
                  weightMatrix[i].append(999)
              else:
                if(y == i+2 or y == i-1):
                  weightMatrix[i].append(1)
                else:
                  weightMatrix[i].append(999)


        f = open(basePath+"pointListOut.tsp", "w")

        # Header
        f.write( 'NAME:%s\n' % "TESTNAME" )
        f.write( 'TYPE:TSP\n' )
        f.write( 'DIMENSION:%d\n' % len( bestPositionsRotated ) )
        f.write( 'EDGE_WEIGHT_TYPE: EXPLICIT\n' )
        f.write( 'EDGE_WEIGHT_FORMAT: FULL_MATRIX\n' )

        # list of weights
        f.write( 'EDGE_WEIGHT_SECTION:\n' )
        for list in weightMatrix:
          msg = ""
          for val in list:
            msg += str(val)+"\t"
          f.write( msg + '\n' )

        # And finally an EOF record
        f.write( 'EOF:\n' )
        f.close()

        solfile = '/Users/h3ct0r/Desktop/concorde/LINKERN/result'
        LINKERN = '/Users/h3ct0r/Desktop/concorde/LINKERN/linkern'
        LINKERN_OPTS = ' -K 1 -q 3 -I 1 -R 1 -r 3 -o %s %s'

        cmd = LINKERN + LINKERN_OPTS % (solfile, "/Users/h3ct0r/Sites/pointListOut.tsp")
        pipe = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, error = pipe.communicate()
        status = pipe.wait()

        if status:
            print('Solver failed; status = %s\n' % status)
            print output, error

        organizedCoordinates = []

        ins = open( solfile, "r" )
        next(ins)
        for line in ins:
          nodeNumber = line.split()
          #print nodeNumber
          print >> sys.stderr, nodeNumber
          organizedCoordinates.append(bestPositionsRotated[int(nodeNumber[0])])
        ins.close()

        print "result coordinates:"
        tsp_coords = []
        for pos in organizedCoordinates:
            tsp_coords.append( (int(pos[0]), int(pos[1])) )
            print int(pos[0]), int(pos[1])

        simulationState = 'SHOW_TSP'

    elif(simulationState == 'SHOW_TSP'):
      pygame.draw.polygon(screen, GREEN, tsp_coords, 1)
      cost = 0
      for i in xrange(1, len(tsp_coords)):
          d, theta = points_to_vector(tsp_coords[i], tsp_coords[i - 1])
          cost += d

      print "total_cost", cost, "len(tsp_coords)", len(tsp_coords)

      d, theta = points_to_vector(clickedPos[0], clickedPos[1])
      print "hex side:", d
      print "hex area:", ((3 * math.sqrt(3)) / 2) * (d ** 2)

      hex_width = math.ceil(2 * d)
      hex_height = math.ceil(math.sqrt(3) * d)
      print "hex width:", hex_width
      print "hex height:", hex_height

      # http://en.wikipedia.org/wiki/Law_of_cosines
      triangle_base = math.sqrt((d ** 2 + d ** 2) - ((2 * (d * d)) * math.cos(math.radians(120))))
      triangle_base = math.ceil(triangle_base)
      print "base of triangle", triangle_base
      dbase, t = points_to_vector(clickedPos[2], clickedPos[4])
      print "   real base", dbase

      triangle_height = (triangle_base / 2) * math.tan(math.radians(30))
      triangle_height = math.ceil(triangle_height)
      print "height of triangle", triangle_height
      xa = clickedPos[4][1] + (triangle_base / 2)
      dbase2, t = points_to_vector((clickedPos[4][0], xa), clickedPos[3])
      print "   real height", dbase2
      pygame.draw.line(screen, RED, (clickedPos[4][0], xa), clickedPos[3])

      num_vertical_lines = int(((d * 2) / lineWidth) + 1)
      print "num_vertical_lines", num_vertical_lines

      counter = 0
      for pos in tsp_coords:
        label = monoFont.render(str(counter), 1, (255, 255, 0))
        screen.blit(label, pos)
        counter += 1

      pygame.draw.polygon(screen, RED, clickedPos, 1)

      lines_in_1 = math.ceil(triangle_height / lineWidth)
      print "number of partitions inside 1:", lines_in_1

      width_of_2 = hex_width - (2 * triangle_height)
      lines_in_2 = math.ceil(width_of_2 / lineWidth)
      print "Width of 2:", width_of_2
      print "number of partitions inside 2:", lines_in_2

      #hex_width triangle_height

      total_1_2 = hex_width - ((((lines_in_2 - 1) + (lines_in_1 - 1)) + 1) * lineWidth)
      print "hex_width", hex_width, "total_1_2", total_1_2


      lines_in_3 = math.ceil(total_1_2 / lineWidth)
      print "number of partitions inside 3:", lines_in_3

      h = lineWidth

      a_first = (2 * h) / math.tan(math.radians(30))
      a_last = (2 * (h * (lines_in_1 - 1))) / math.tan(math.radians(30))
      first_term = math.ceil(((lines_in_1 - 1) * (a_first + a_last)) / 2)

      second_term = (hex_height * (lines_in_2 - 1))

      print "a ", (total_1_2 - (h * (lines_in_3 - 1)))
      print "b ", total_1_2

      b_first = (2 * (total_1_2 - (h * (lines_in_3 - 1)))) / math.tan(math.radians(30))
      b_last = (2 * total_1_2) / math.tan(math.radians(30))
      third_term = math.ceil( ((lines_in_3) * (b_first + b_last)) / 2)

      total_calculated = first_term + second_term + third_term

      print "first_term", first_term, "second_term", second_term, "third term", third_term

      rest_side = math.sqrt(((b_first/2) ** 2) + ((total_1_2 - (h * (lines_in_3 - 1))) ** 2))

      print "rest_side", rest_side

      hex_lawnmower_perimeter = total_calculated + ((d * 3) - rest_side)

      print "total_calculated", hex_lawnmower_perimeter
      print "total_cost waypoint", cost
      print "size", lineWidth, "with_geometry", hex_lawnmower_perimeter, "via waypoint", cost
      print "-----------------"

      #f_line, t = points_to_vector(tsp_coords[3], tsp_coords[2])
      #print "   real base first line", f_line

      simulationState = 'GENERATE_SQUARE'

    elif(simulationState == 'GENERATE_SQUARE'):

      #centroid
      border_polygon = Polygon(clickedPos)

      #border_polygon.centroid.x
      #border_polygon.centroid.y

      c = (int(border_polygon.centroid.x), int(border_polygon.centroid.y))

      # Distance to the first point
      dx, dy = clickedPos[0][0] - c[0], clickedPos[0][1] - c[1]
      mag = math.sqrt(dx ** 2 + dy ** 2)

      #print "mag:", mag

      pygame.draw.polygon(screen, GREEN, tsp_coords, 1)
      pygame.draw.circle(screen, WHITE, c, 3)

      angle_step = 360 / 4

      square_coords = []
      for i in xrange(4):
          actual_step = (angle_step * i)

          x, y = vector_components((mag * 0.6), actual_step * 3.14/180)
          new_pos = (int(c[0] + x), int(c[1] + y))
          square_coords.append(new_pos)

      simulationState = "DRAW_ALL"
      pass

    elif(simulationState == "DRAW_ALL"):


        pygame.draw.polygon(screen, GREEN, tsp_coords, 1)
        pygame.draw.polygon(screen, RED, square_coords, 1)

        pygame.draw.circle(screen, BLUE, tsp_coords[0], 3)
        pygame.draw.circle(screen, WHITE, tsp_coords[-1], 3)

        counter = 1
        for pos in tsp_coords:
            label = monoFont.render(str(counter), 1, (255,255,0))
            screen.blit(label, pos)
            counter += 1


        pygame.draw.polygon(screen, RED, clickedPos, 1)
        border_polygon = Polygon(clickedPos)
        c = (int(border_polygon.centroid.x), int(border_polygon.centroid.y))
        pygame.draw.circle(screen, WHITE, c, 1)

        """
        counter = 1
        for pos in bestPositionsRotated:
            label = monoFont.render(str(counter), 1, (255,255,0))
            screen.blit(label, pos)
            counter += 1
        """
        done = True

    pygame.draw.rect(screen, BLACK, [0, sizeY - 25, sizeX, 25])
    label = monoFont.render("lineAngle:" + str(lineAngle) + " Best angle: "+str(bestAngle)+" best Size: "+str(bestSize)+" Actualpos :"+str(pygame.mouse.get_pos()), 1, (255,255,0))
    screen.blit(label, (10, sizeY - 25))

    clock.tick()
    pygame.display.set_caption("fps:" + str(clock.get_fps()))

    #clock.tick(100) # Limit to 20 FPS
    pygame.display.flip()

    #pygame.time.delay(10)

#print int(organizedCoordinates[0][0]), int(organizedCoordinates[0][1])

# Print coords to file
f = open("/tmp/hex_coords_calculated.txt", "w")
for pos in tsp_coords:
    line = str(pos[0]) + " " + str(pos[1]) + "\n"
    f.write(line)
f.close()

f = open("/tmp/square_coords_calculated.txt", "w")
for pos in square_coords:
    line = str(pos[0]) + " " + str(pos[1]) + "\n"
    f.write(line)
f.close()

time.sleep(0)
pygame.quit()