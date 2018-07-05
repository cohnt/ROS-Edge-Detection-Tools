#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# https://docs.opencv.org/3.4.1/da/d22/tutorial_py_canny.html
from matplotlib import pyplot as plt
import numpy as np

from skimage.feature import hog
from skimage.color import rgb2gray
from skimage import data, exposure, io
from skimage.io import imread

# Instantiate CvBridge
bridge = CvBridge()

first = True
busy = False

plt.ion()
# plt.show(block=False)

fig = plt.figure()

ax1 = fig.add_subplot(111)

im1 = 0

# Hog config
cell_size = (8, 8) # x, y

def image_callback(msg):
    global first, im1, busy
    print("Received an image!")
    if busy:
        return
    else:
        print("Processing...")
        busy = True
    try:
        # Convert your ROS Image message to OpenCV2
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        # cv2.imwrite('camera_image.jpeg', img)

        fd, hog_image = hog(rgb2gray(img), orientations=8, pixels_per_cell=cell_size, cells_per_block=(1, 1), visualise=True, feature_vector=False, block_norm='L2')

        # ax1.axis('off')
        # ax1.imshow(image, cmap=plt.cm.gray)
        # ax1.set_title('Input image')

        # # Rescale histogram for better display
        hog_image_rescaled = exposure.rescale_intensity(hog_image, in_range=(0, 10))

        # ax2.axis('off')
        # ax2.imshow(hog_image_rescaled, cmap=plt.cm.gray)
        # ax2.set_title('Histogram of Oriented Gradients')

        if first:
            im1 = ax1.imshow(hog_image_rescaled, cmap='gray')
            first = False
        else:
            im1.set_data(hog_image_rescaled)

        cv2.imwrite('hog.jpeg', hog_image_rescaled)

        # ax1.relim()
        # ax1.autoscale_view(True,True,True)
        # ax2.relim()
        # ax2.autoscale_view(True,True,True)

        # plt.subplot(121),plt.imshow(img,cmap = 'gray')
        # plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        # plt.subplot(122),plt.imshow(edges,cmap = 'gray')
        # plt.title('Edge Image'), plt.xticks([]), plt.yticks([])

    busy = False
    return

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/head_camera/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback, queue_size=1)
    # Spin until ctrl + c
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    while not rospy.is_shutdown():
        try:
            fig.canvas.draw()
        except KeyboardInterrupt:
            break

if __name__ == '__main__':
    main()