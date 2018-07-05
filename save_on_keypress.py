#! /usr/bin/python

# rospy for the subscriber
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Instantiate CvBridge
bridge = CvBridge()

imNum = 0
lastImNum = -1

# Keylogger from StackOverflow
# https://stackoverflow.com/questions/13207678/whats-the-simplest-way-of-detecting-keyboard-input-in-python-from-the-terminal
import sys
import select
import termios

class KeyPoller():
    def __enter__(self):
        # Save the terminal settings
        self.fd = sys.stdin.fileno()
        self.new_term = termios.tcgetattr(self.fd)
        self.old_term = termios.tcgetattr(self.fd)

        # New terminal setting unbuffered
        self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)

        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)

    def poll(self):
        dr,dw,de = select.select([sys.stdin], [], [], 0)
        if not dr == []:
            return sys.stdin.read(1)
        return None

def image_callback(msg):
    global imNum, lastImNum
    # print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        print imNum
        if imNum != lastImNum:
            cv2.imwrite('camera_image%s.jpeg' % imNum, cv2_img)
            lastImNum = imNum


def main():
    global imNum
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/head_camera/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    with KeyPoller() as keyPoller:
        while not rospy.is_shutdown():
            c = keyPoller.poll()
            if c == 'q':
                print "Saved %d images." % imNum
                rospy.signal_shutdown("Quit command received. Stopping...")
                break
            elif c == 'r':
                imNum = imNum + 1
                print "\tSaving an image."

if __name__ == '__main__':
    main()