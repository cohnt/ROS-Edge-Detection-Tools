#!/usr/bin/env python2

import matplotlib.pyplot as plt
import matplotlib.widgets as widgets

from skimage.feature import hog
from skimage.color import rgb2gray
from skimage import data, exposure, io
from skimage.io import imread

import numpy as np

np.set_printoptions(threshold='nan')

fname = "camera_image.jpeg"
cell_size = (8, 8)
window_size = (18, 12)

fig, ax = plt.subplots()
ax.axis('off')

class WindowIndicator(object):
	def __init__(self, ax):
		self.ax = ax
		self.vl1 = ax.axvline(color='red', alpha=0.75)
		self.vl2 = ax.axvline(color='red', alpha=0.75)
		self.hl1 = ax.axhline(color='red', alpha=0.75)
		self.hl2 = ax.axhline(color='red', alpha=0.75)
		self.x = 0
		self.y = 0
	
	def mouse_move(self, event):
		if not event.inaxes: return
		x, y = event.xdata, event.ydata
		self.vl1.set_xdata(x-(x % cell_size[0]))
		self.vl2.set_xdata(x+(cell_size[0]*window_size[0])-(x % cell_size[0]))
		self.hl1.set_ydata(y-(y % cell_size[1]))
		self.hl2.set_ydata(y+(cell_size[1]*window_size[1])-(y % cell_size[1]))
		self.ax.figure.canvas.draw_idle()

def onclick(event):
	if ax.in_axes(event):
		# Transform the event from display to axes coordinates
		ax_pos = ax.transAxes.inverted().transform((event.x, event.y))
		exact_pos = ax_pos * (640, 480)
		exact_pos[1] = 480 - exact_pos[1]
		exact_pos = exact_pos - (0.5, 0.5)
		print(exact_pos)
		if event.button == 1:
			print "NOT HANDLE"
		elif event.button == 3:
			print "HANDLE"
	else:
		print("Outside of figure!")

def onkeypress(event):
	# print event.key
	if event.key == "enter":
		print "Next image!"

def main():
	image = imread(fname)
	fd, hog_image = hog(rgb2gray(image), orientations=8, pixels_per_cell=cell_size, cells_per_block=(1, 1), visualise=True, feature_vector=False, block_norm='L2')
	hog_image_rescaled = exposure.rescale_intensity(hog_image, in_range=(0, 10))

	ax.imshow(hog_image_rescaled, cmap=plt.cm.gray)

	cursor = WindowIndicator(ax)
	cid1 = plt.connect('motion_notify_event', cursor.mouse_move)
	cid2 = plt.connect('button_press_event', onclick)
	cid3 = plt.connect('key_press_event', onkeypress)

	mng = plt.get_current_fig_manager()
	mng.resize(*mng.window.maxsize())

	plt.show()


if __name__ == '__main__':
	main()