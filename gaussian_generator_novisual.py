import numpy as np
import random

def gauss2D(size=10, sigma=1):
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

    return g

IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600

is_loop_active = True
redraw = True

mat_output = np.zeros((600, 800))

for i in xrange(random.randint(10, 20)):
    size = random.randint(200, 400)
    sigma = size * random.uniform(0.09, 0.2)
    g = gauss2D(size, sigma)

    center = (random.randint(0, IMAGE_WIDTH), random.randint(0, IMAGE_HEIGHT))

    for x in xrange(g.shape[0]):
        for y in xrange(g.shape[1]):
            pos = (center[0] + x, center[1] + y)

            if IMAGE_WIDTH > pos[0] >= 0 and IMAGE_HEIGHT > pos[1] >= 0:
                mat_output[pos[1]][pos[0]] = mat_output[pos[1]][pos[0]] + g[y, x]
        pass
    pass


mat_white_noise_output = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH))
for x in xrange(IMAGE_WIDTH):
    for y in xrange(IMAGE_HEIGHT):
        noise = np.random.normal(mat_output[y][x], 0.2, 1)
        #print noise
        mat_white_noise_output[y][x] = noise[0]
    pass
pass
np.savetxt('/tmp/magnetic_ground_truth.np', mat_output)
np.savetxt('/tmp/magnetic_white_noise.np', mat_white_noise_output)

print mat_output
print mat_white_noise_output