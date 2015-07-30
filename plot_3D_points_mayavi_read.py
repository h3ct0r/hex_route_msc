import numpy as np
from mayavi import mlab

WIDTH = 800
HEIGHT = 600

x = np.linspace(0, WIDTH, 1)
y = np.linspace(0, HEIGHT, 1)
xx, yy = np.meshgrid(x, y)

z = np.loadtxt('/tmp/magnetic_ground_truth.np')
mlab.figure(bgcolor=(1,1,1))

# We'll use "surf" to display a 2D grid...
# warp_scale='auto' guesses a vertical exaggeration for better display.
# Feel free to remove the "warp_scale" argument for "true" display.
mlab.surf(x, y, z, warp_scale='auto')

mlab.show()

z = np.loadtxt('/tmp/magnetic_white_noise.np')
mlab.figure(bgcolor=(1,1,1))

# We'll use "surf" to display a 2D grid...
# warp_scale='auto' guesses a vertical exaggeration for better display.
# Feel free to remove the "warp_scale" argument for "true" display.
mlab.surf(x, y, z, warp_scale='auto')

mlab.show()