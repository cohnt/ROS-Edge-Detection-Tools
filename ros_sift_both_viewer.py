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

firstIm = True
firstD = True

plt.ion()
# plt.show(block=False)

fig = plt.figure()

ax1 = fig.add_subplot(121)
ax2 = fig.add_subplot(122)

im1 = 0
im2 = 0

new_img1 = 0
new_img2 = 0

sift = cv2.xfeatures2d.SIFT_create()

def image_callback(msg):
    global firstIm, im1, new_img1
    print("Received an RGB image!")
    try:
        # Convert your ROS Image message to OpenCV2
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kp, des = sift.detectAndCompute(gray, None)
        new_img1 = cv2.drawKeypoints(gray, kp, new_img1, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        # cv2.imwrite('camera_image.jpeg', img)

        if firstIm:
            im1 = ax1.imshow(new_img1)
            firstIm = False
        else:
            im1.set_data(new_img1)

        cv2.imwrite('sift_rgb.png', new_img1)


def depth_callback(msg):
    global firstD, im2, new_img2
    print("Received a depth image!")
    try:
        # Convert your ROS Image message to OpenCV2
        raw_img = bridge.imgmsg_to_cv2(msg, "16UC1")
        print "max " + str(raw_img.max())
        # Use this site to come up with convertion curves:
        # https://mycurvefit.com/
        # y = 255.1347*e^(-(x - -21.62828)^2/(2*746.8639^2))

        # raw_img[raw_img==0] = 65535
        # raw_img = 255.1347*(2.7**(-(((raw_img + 21.62828)**2)/(2*746.8639**2))))

        # OLDER EQUATIONS
        raw_img = 255-((0.226632/0.0008148313)*(1-(2.7**(-0.0008148313*raw_img))))

        # OLD METHOD
        # raw_img = raw_img * -1
        # raw_img = raw_img * 0.5
        # raw_img = 1.001**raw_img
        # raw_img = raw_img * 255

        img = np.dstack((raw_img, raw_img, raw_img))
        img = np.uint8(img)
        # img = cv2.cvtColor(raw_img, cv2.COLOR_GRAY2RGB)
        # img = increase_brightness(img, value=40)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kp, des = sift.detectAndCompute(gray, None)
        new_img2 = cv2.drawKeypoints(gray, kp, new_img2, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        # cv2.imwrite('camera_image.jpeg', img)
        edges = cv2.Canny(img,10,20)

        if firstD:
            im2 = ax2.imshow(new_img2)
            firstD = False
        else:
            im2.set_data(new_img2)

        cv2.imwrite('sift_depth.png', new_img2)

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/head_camera/rgb/image_raw"
    depth_topic = "/head_camera/depth/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.Subscriber(depth_topic, Image, depth_callback)
    # Spin until ctrl + c
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    while True:
        fig.canvas.draw()

if __name__ == '__main__':
    main()