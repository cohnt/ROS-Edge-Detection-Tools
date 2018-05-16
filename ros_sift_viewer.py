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

# Instantiate CvBridge
bridge = CvBridge()

first = True

plt.ion()
# plt.show(block=False)

fig = plt.figure()

ax1 = fig.add_subplot(111)

im1 = 0
new_img = 0

def image_callback(msg):
    global first, im1, im2, new_img
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        sift = cv2.xfeatures2d.SIFT_create()
        kp, des = sift.detectAndCompute(gray, None)
        new_img = cv2.drawKeypoints(gray, kp, new_img, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        if first:
            im1 = ax1.imshow(new_img)
            first = False
        else:
            im1.set_data(new_img)

        cv2.imwrite('sift_image.png', new_img)

        # ax1.relim()
        # ax1.autoscale_view(True,True,True)
        # ax2.relim()
        # ax2.autoscale_view(True,True,True)

        # plt.subplot(121),plt.imshow(img,cmap = 'gray')
        # plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        # plt.subplot(122),plt.imshow(edges,cmap = 'gray')
        # plt.title('Edge Image'), plt.xticks([]), plt.yticks([])

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/head_camera/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    while True:
        fig.canvas.draw()

if __name__ == '__main__':
    main()