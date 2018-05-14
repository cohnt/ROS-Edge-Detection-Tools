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
firstO = True

plt.ion()
# plt.show(block=False)

fig = plt.figure()

ax1 = fig.add_subplot(111) # Overlaid

im1 = 0
im2 = 0
im3 = 0
im4 = 0
im5 = 0

img = 0

np.set_printoptions(threshold='nan')

def update_overlaid():
    global firstIm, firstD, im2, im4
    print("Updating overlaid edges image")

    if firstIm or firstD:
        return
    if firstO:
        img = im1

    mask1 = im2 > 127
    mask2 = im4 > 127
    img[np.logical_and(mask1, mask2)] = (255, 0, 255)
    img[np.logical_and(mask1, np.logical_not(mask2))] = (0, 0, 255)
    img[np.logical_and(np.logical_not(mask1), mask2)] = (255, 0, 0)
    img[np.logical_and(np.logical_not(mask1), np.logical_not(mask2))] = (0, 0, 0)

    if firstO:
        im5 = ax1.imshow(img, cmap='gray')
    else:
        im5.set_data(img)

    cv2.imwrite('overlay_test.png', img)

def image_callback(msg):
    global firstIm, im1, im2
    print("Received an RGB image!")
    try:
        # Convert your ROS Image message to OpenCV2
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        # cv2.imwrite('camera_image.jpeg', img)
        edges = cv2.Canny(img,100,200)

        if firstIm:
            # im1 = ax1.imshow(img, cmap='gray')
            # im2 = ax2.imshow(edges, cmap='gray')
            firstIm = False
        else:
            im1 = img
            im2 = edges
            # im1.set_data(img)
            # im2.set_data(edges)

        cv2.imwrite('camera_image.jpeg', img)
        cv2.imwrite('camera_edges.jpeg', edges)

        update_overlaid()

        # ax1.relim()
        # ax1.autoscale_view(True,True,True)
        # ax2.relim()
        # ax2.autoscale_view(True,True,True)

        # plt.subplot(121),plt.imshow(img,cmap = 'gray')
        # plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        # plt.subplot(122),plt.imshow(edges,cmap = 'gray')
        # plt.title('Edge Image'), plt.xticks([]), plt.yticks([])

# https://stackoverflow.com/questions/32609098/how-to-fast-change-image-brightness-with-python-opencv
# def increase_brightness(img, value=30):
#     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#     h, s, v = cv2.split(hsv)

#     lim = 255 - value
#     v[v > lim] = 255
#     v[v <= lim] += value

#     final_hsv = cv2.merge((h, s, v))
#     img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
#     return img

def depth_callback(msg):
    global firstD, im3, im4
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
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        # cv2.imwrite('camera_image.jpeg', img)
        edges = cv2.Canny(img,10,20)

        if firstD:
            # im3 = ax3.imshow(img, cmap='gray')
            # im4 = ax4.imshow(edges, cmap='gray')
            firstD = False
        else:
            im3 = img
            im4 = edges
            # im3.set_data(img)
            # im4.set_data(edges)

        cv2.imwrite('depth_image.jpeg', img)
        cv2.imwrite('depth_edges.jpeg', edges)

        update_overlaid()

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