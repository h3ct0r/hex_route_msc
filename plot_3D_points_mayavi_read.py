import numpy as np
from mayavi import mlab
import scipy.signal
from scipy.ndimage.filters import gaussian_filter
from skimage.measure import structural_similarity as ssim
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import norm


def mse(imageA, imageB):
    # the 'Mean Squared Error' between the two images is the
    # sum of the squared difference between the two images;
    # NOTE: the two images must have the same dimension
    err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
    err /= float(imageA.shape[0] * imageA.shape[1])

    # return the MSE, the lower the error, the more "similar"
    # the two images are
    return err


def compare_images(imageA, imageB, title):
    # compute the mean squared error and structural similarity
    # index for the images
    m = mse(imageA, imageB)
    s = ssim(imageA, imageB)

    # setup the figure
    #fig = plt.figure(title)
    #plt.suptitle("MSE: %.2f, SSIM: %.2f" % (m, s))
    print "MSE: %.5f, SSIM: %.5f" % (m, s)

    # show first image
    #ax = fig.add_subplot(1, 2, 1)
    #plt.imshow(imageA, cmap = plt.cm.gray)
    #plt.axis("off")

    # show the second image
    #ax = fig.add_subplot(1, 2, 2)
    #plt.imshow(imageB, cmap = plt.cm.gray)
    #plt.axis("off")

    # show the images
    #plt.show()


def compare_images2(img1, img2):
    # calculate the difference and its norms
    diff = img1 - img2  # elementwise for scipy arrays
    m_norm = sum(abs(diff))  # Manhattan norm
    z_norm = norm(diff.ravel(), 0)  # Zero norm
    return (m_norm, z_norm)


def psnr(dataset1, dataset2, maximumDataValue, ignore=None):
   """
   title::
      psnr

   description::
      This method will compute the peak-signal-to-noise ratio (PSNR) between
      two provided data sets.  The PSNR will be computed for the ensemble
      data.  If the PSNR is desired for a particular slice of the provided
      data, then the data sets provided should represent those slices.

   attributes::
      dataset1
         An array-like object containing the first data set.
      dataset2
         An array-like object containing the second data set.
      maximumDataValue
         The maximum value that might be contained in the data set (this
         is not necessaryily the the maximum value in the data set, but
         rather it is the largest value that any member of the data set
         might take on).
      ignore
         A scalar value that will be ignored in the data sets.  This can
         be used to mask data in the provided data set from being
         included in the analysis. This value will be looked for in both
         of the provided data sets, and only an intersection of positions
         in the two data sets will be included in the computation. [default
         is None]

   author::
      Carl Salvaggio

   copyright::
      Copyright (C) 2015, Rochester Institute of Technology

   license::
      GPL

   version::
      1.0.0

   disclaimer::
      This source code is provided "as is" and without warranties as to
      performance or merchantability. The author and/or distributors of
      this source code may have made statements about this source code.
      Any such statements do not constitute warranties and shall not be
      relied on by the user in deciding whether to use this source code.

      This source code is provided without any express or implied warranties
      whatsoever. Because of the diversity of conditions and hardware under
      which this source code may be used, no warranty of fitness for a
      particular purpose is offered. The user is advised to test the source
      code thoroughly before relying on it. The user must assume the entire
      risk of using the source code.
   """

   # Make sure that the provided data sets are numpy ndarrays, if not
   # convert them and flatten te data sets for analysis
   if type(dataset1).__module__ != np.__name__:
      d1 = np.asarray(dataset1).flatten()
   else:
      d1 = dataset1.flatten()

   if type(dataset2).__module__ != np.__name__:
      d2 = np.asarray(dataset2).flatten()
   else:
      d2 = dataset2.flatten()

   # Make sure that the provided data sets are the same size
   if d1.size != d2.size:
      raise ValueError('Provided datasets must have the same size/shape')

   # Check if the provided data sets are identical, and if so, return an
   # infinite peak-signal-to-noise ratio
   if np.array_equal(d1, d2):
      return float('inf')

   # If specified, remove the values to ignore from the analysis and compute
   # the element-wise difference between the data sets
   if ignore is not None:
      index = np.intersect1d(np.where(d1 != ignore)[0],
                                np.where(d2 != ignore)[0])
      error = d1[index].astype(np.float64) - d2[index].astype(np.float64)
   else:
      error = d1.astype(np.float64)-d2.astype(np.float64)

   # Compute the mean-squared error
   meanSquaredError = np.sum(error**2) / error.size

   # Return the peak-signal-to-noise ratio
   return 10.0 * np.log10(maximumDataValue**2 / meanSquaredError)


WIDTH = 800
HEIGHT = 600

x = np.linspace(0, WIDTH, 1)
y = np.linspace(0, HEIGHT, 1)
xx, yy = np.meshgrid(x, y)

z = np.loadtxt('/tmp/magnetic_ground_truth.np')
mlab.figure(bgcolor=(1,1,1), fgcolor=(0,0,0))
mlab.surf(x, y, z, warp_scale='auto')
mlab.colorbar(title='Original', orientation='vertical')

original = z

# z = np.loadtxt('/tmp/magnetic_white_noise.np')
# mlab.figure(bgcolor=(1,1,1))
# mlab.surf(x, y, z, warp_scale='auto')

# z = np.loadtxt('/tmp/magnetic_white_noise.np')
# blurred = gaussian_filter(z, sigma=45)
# mlab.figure(bgcolor=(1,1,1))
# mlab.surf(x, y, blurred, warp_scale='auto')

#print "Original:    ", mse(original, original)
#print "Zeros:       ", mse(original, np.zeros((600, 800)))
#print "Ones:        ", mse(original, (np.zeros((600, 800)) + 1))

z = np.loadtxt('/tmp/magnetic_sampled.np')
blurred = gaussian_filter(z, sigma=45)
mlab.figure(bgcolor=(1,1,1), fgcolor=(0,0,0))
mlab.surf(x, y, blurred, warp_scale='auto')
mlab.colorbar(title='Metodologia', orientation='vertical')

print "Metodology:  "#, compare_images(original, blurred, "metodology")
#n_m, n_0 = compare_images2(original, blurred)
#print "Manhattan norm:", n_m, "/ per pixel:", n_m/ 800*600
#print "Zero norm:", n_0, "/ per pixel:", n_0*1.0/ 800*600
#print "Mean:", np.mean( original != blurred )
#print "Sum:", np.sum( original - blurred )
#print "psnr:", psnr(original, blurred, 1)

z = np.loadtxt('/tmp/magnetic_sampled_ollero.np')
blurred = gaussian_filter(z, sigma=45)
mlab.figure(bgcolor=(1,1,1))
mlab.surf(x, y, blurred, warp_scale='auto')
mlab.colorbar(title='Ollero', orientation='vertical')

print "Ollero:      "#, compare_images(original, blurred, "ollero")
#n_m, n_0 = compare_images2(original, blurred)
#print "Manhattan norm:", n_m, "/ per pixel:", n_m/ 800*600
#print "Zero norm:", n_0, "/ per pixel:", n_0*1.0/ 800*600
#print "Mean:", np.mean( original != blurred )
#print "Sum:", np.sum( original - blurred )
#print "psnr:", psnr(original, blurred, 1)

mlab.show()
