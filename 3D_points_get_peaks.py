import os
import sys
sys.path.insert(0, '/Library/Python/2.7/site-packages/')

from mayavi import mlab
from scipy.ndimage.filters import gaussian_filter
from skimage.measure import structural_similarity as ssim
import numpy as np
from scipy.linalg import norm
import inspect

WIDTH = 800
HEIGHT = 600

x = np.linspace(0, WIDTH, 1)
y = np.linspace(0, HEIGHT, 1)
xx, yy = np.meshgrid(x, y)

z = np.loadtxt('/tmp/magnetic_ground_truth.np')
# mlab.figure(bgcolor=(1,1,1), fgcolor=(0,0,0))
# mlab.surf(x, y, z, warp_scale='auto')
# mlab.colorbar(title='Original', orientation='vertical')

original = z

original_nonzero_indices = np.transpose((original > 0).nonzero())
#print original_nonzero_indices

values_original = np.zeros(original_nonzero_indices.shape[0])
for i in xrange(original_nonzero_indices.shape[0]):
    pos = original_nonzero_indices[i]
    values_original[i] = original[pos[0]][pos[1]]
    pass

#print values_original

#===--------------------===#
#===     Metodology     ===#
#===--------------------===#
z = np.loadtxt('/tmp/magnetic_sampled.np')
blurred = gaussian_filter(z, sigma=45)
blurred *= (original.max()/blurred.max())

mlab.figure(bgcolor=(1,1,1), fgcolor=(0,0,0))
mlab.surf(x, y, blurred, warp_scale='auto')
mlab.colorbar(title='Metodologia', orientation='vertical')

values_metodology = np.zeros(original_nonzero_indices.shape[0])
for i in xrange(original_nonzero_indices.shape[0]):
    pos = original_nonzero_indices[i]
    values_metodology[i] = blurred[pos[0]][pos[1]]
    pass

print "Mse methodology:", ((values_original - values_metodology) ** 2).mean()

#===--------------------===#
#===       Ollero       ===#
#===--------------------===#
z = np.loadtxt('/tmp/magnetic_sampled_ollero.np')
blurred = gaussian_filter(z, sigma=45)
blurred *= (original.max()/blurred.max())

mlab.figure(bgcolor=(1,1,1), fgcolor=(0,0,0))
mlab.surf(x, y, blurred, warp_scale='auto')
mlab.colorbar(title='Ollero', orientation='vertical')

values_ollero = np.zeros(original_nonzero_indices.shape[0])
for i in xrange(original_nonzero_indices.shape[0]):
    pos = original_nonzero_indices[i]
    values_ollero[i] = blurred[pos[0]][pos[1]]
    pass

print "Mse ollero:", ((values_original - values_ollero) ** 2).mean()

mlab.show()