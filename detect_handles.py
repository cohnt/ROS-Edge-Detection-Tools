#!/usr/bin/env python2

import matplotlib.pyplot as plt
import matplotlib.widgets as widgets

from skimage.feature import hog
from skimage.color import rgb2gray
from skimage import data, exposure, io
from skimage.io import imread

from sklearn.svm import LinearSVC
from sklearn.datasets import make_classification

import numpy as np

# np.set_printoptions(threshold='nan')

fname = "camera_image.jpeg"
cell_size = (8, 8)
window_size = (18, 12)

fig, ax = plt.subplots()
ax.axis('off')

isHandleData = 0
notHandleData = 0

fd = 0

svc = 0
learned = False

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
		self.hl1.set_ydata(y-(y % cell_size[1])-1)
		self.hl2.set_ydata(y+(cell_size[1]*window_size[1])-(y % cell_size[1])-1)
		self.ax.figure.canvas.draw_idle()

def classify(cell_id):
	global fd, svc

def onclick(event):
	global fd, isHandleData, notHandleData, svc, cell_size, window_size
	if ax.in_axes(event):
		# Transform the event from display to axes coordinates
		ax_pos = ax.transAxes.inverted().transform((event.x, event.y))
		exact_pos = ax_pos * (640, 480)
		exact_pos[1] = 480 - exact_pos[1]
		exact_pos = exact_pos - (0.5, 0.5)
		print(exact_pos)
		cell_id = np.copy(exact_pos)
		cell_id[0] = np.floor(exact_pos[1] / cell_size[1])
		cell_id[1] = np.floor(exact_pos[0] / cell_size[0])
		cell_id = cell_id.astype(int)
		print(cell_id)
		if event.button == 1:
			print "NOT HANDLE"
			if learned:
				classify(cell_id)
			else:
				if np.shape(notHandleData) == ():
					notHandleData = [fd[cell_id[0]:cell_id[0]+window_size[1],cell_id[1]:cell_id[1]+window_size[0],:,:,:]]
				else:
					notHandleData = np.append(notHandleData, [fd[cell_id[0]:cell_id[0]+window_size[1],cell_id[1]:cell_id[1]+window_size[0],:,:,:]], axis=0)
		elif event.button == 3:
			print "HANDLE"
			if learned:
				classify(cell_id)
			else:
				if np.shape(isHandleData) == ():
					isHandleData = [fd[cell_id[0]:cell_id[0]+window_size[1],cell_id[1]:cell_id[1]+window_size[0],:,:,:]]
				else:
					isHandleData = np.append(isHandleData, [fd[cell_id[0]:cell_id[0]+window_size[1],cell_id[1]:cell_id[1]+window_size[0],:,:,:]], axis=0)
	else:
		print("Outside of figure!")

def learn():
	global isHandleData, notHandleData, svc
	svc = LinearSVC(random_state=0)
	positives = np.reshape(isHandleData, (np.shape(isHandleData)[0], window_size[0], window_size[1], np.shape(isHandleData)[5]))
	positives = np.reshape(positives, (np.shape(positives)[0], -1))
	negatives = np.reshape(notHandleData, (np.shape(notHandleData)[0], window_size[0], window_size[1], np.shape(notHandleData)[5]))
	negatives = np.reshape(negatives, (np.shape(negatives)[0], -1))

	print np.shape(positives)
	print np.shape(negatives)

def onkeypress(event):
	global isHandleData, notHandleData, svc
	# print event.key
	if event.key == "enter":
		print "Next image!"
	elif event.key == "h": # Print handles
		# print isHandleData
		print "Positive examples: %s" % np.shape(isHandleData)[0]
	elif event.key == "n": # Print negatives
		# print notHandleData
		print "Negative examples: %s" % np.shape(notHandleData)[0]
	elif event.key == "l": # Learn
		learned = True
		learn()

def main():
	global fd
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

	plt.rcParams["keymap.yscale"] = ""

	plt.show()


if __name__ == '__main__':
	main()