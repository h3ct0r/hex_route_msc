import numpy as np
import pygame
import random
import pylab as plt

def twoD_Gaussian((x, y), amplitude, xo, yo, sigma_x, sigma_y, theta, offset):
    xo = float(xo)
    yo = float(yo)
    a = (np.cos(theta)**2)/(2*sigma_x**2) + (np.sin(theta)**2)/(2*sigma_y**2)
    b = -(np.sin(2*theta))/(4*sigma_x**2) + (np.sin(2*theta))/(4*sigma_y**2)
    c = (np.sin(theta)**2)/(2*sigma_x**2) + (np.cos(theta)**2)/(2*sigma_y**2)
    g = offset + amplitude*np.exp( - (a*((x-xo)**2) + 2*b*(x-xo)*(y-yo)
                            + c*((y-yo)**2)))
    return g.ravel()

def gauss2D(size=10, sigma=1, normalize=255.0):
    """
    Generate a 2D gaussian from size and sigma
    """
    zeroes = (size, size)
    g = np.zeros(zeroes)

    for i in xrange(-(size - 1)/2, (size - 1)/2):
        for j in xrange(-(size - 1)/2, (size - 1)/2):
            x0 = (size + 1)/2
            y0 = (size + 1)/2
            x = i + x0
            y = j + y0
            g[y, x] = np.exp( -( pow((x-x0), 2) + pow((y-y0), 2)) / (2.*sigma*sigma) )
        pass
    pass

    #g *= (normalize/g.max())

    return g

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
redraw = True

all_gaussians = np.zeros((600, 800))

while is_loop_active:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            is_loop_active = False
        elif (event.type == pygame.KEYDOWN) and (event.key == pygame.K_SPACE):
            redraw = True

    if redraw:
        screen.fill(BLACK)
        all_gaussians = np.zeros((600, 800))

        for i in xrange(random.randint(5,10)):
            size = random.randint(200, 400)
            #sigma = size * 0.09
            sigma = size * random.uniform(0.09, 0.2)
            g = gauss2D(size, sigma)

            # x = np.linspace(0, size, size)
            # y = np.linspace(0, size, size)
            # x, y = np.meshgrid(x, y)
            # g = twoD_Gaussian((x, y), random.randint(2, 40), 100, 100, size * 0.09, size * 0.09, 0, 10)
            # g *= (255/g.max())
            # g = g.reshape(size, size)

            center = (random.randint(0, IMAGE_WIDTH), random.randint(0, IMAGE_HEIGHT))

            for x in xrange(g.shape[0]):
                for y in xrange(g.shape[1]):
                    pos = (center[0] + x, center[1] + y)

                    if IMAGE_WIDTH > pos[0] >= 0 and IMAGE_HEIGHT > pos[1] >= 0:
                        value_at_pos = screen.get_at(pos)
                        new_value = value_at_pos[2] + int(g[x, y])
                        if new_value > 255:
                            new_value = 255
                        screen.set_at(pos, (0, 0, new_value))
                pass
            pass

        mat_output = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH))
        mat_white_noise_output = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH))
        # Write to file
        for x in xrange(IMAGE_WIDTH):
            for y in xrange(IMAGE_HEIGHT):
                pos = (x, y)
                value_at_pos = screen.get_at(pos)
                mat_output[y][x] = value_at_pos[2]

                noise = np.random.normal(value_at_pos[2], 10, 1)
                mat_white_noise_output[y][x] = noise[0]
            pass
        pass
        np.savetxt('/tmp/magnetic_ground_truth.np', mat_output)
        np.savetxt('/tmp/magnetic_white_noise.np', mat_white_noise_output)
        redraw = False

    clock.tick(30)
    pygame.display.flip()
    pygame.display.set_caption('FPS: ' + str(clock.get_fps()))

    pass