#!/usr/bin/env python2

import matplotlib.pyplot as plt

from skimage.feature import hog
from skimage.color import rgb2gray
from skimage import data, exposure, io
from skimage.io import imread

fname = "camera_image.jpeg"
cell_size = (16, 16)

image = imread(fname)

fd, hog_image = hog(rgb2gray(image), orientations=8, pixels_per_cell=cell_size, cells_per_block=(1, 1), visualise=True)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 4), sharex=True, sharey=True)

ax1.axis('off')
ax1.imshow(image, cmap=plt.cm.gray)
ax1.set_title('Input image')

# Rescale histogram for better display
hog_image_rescaled = exposure.rescale_intensity(hog_image, in_range=(0, 10))

ax2.axis('off')
ax2.imshow(hog_image_rescaled, cmap=plt.cm.gray)
ax2.set_title('Histogram of Oriented Gradients')
plt.show()