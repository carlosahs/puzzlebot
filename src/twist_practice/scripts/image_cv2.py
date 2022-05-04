#!/usr/bin/env python3
from __future__ import print_function

import roslib
import numpy as np
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from skimage.color import rgb2gray, gray2rgb


class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
        self.pub_twist = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("video_source/raw", Image, self.callback)

        # Variables
        self.vel = Twist()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Color RED
        red_lower = np.array([136, 87, 111], np.uint8)  # 136,87,111
        red_upper = np.array([180, 255, 255], np.uint8)  # 180,255,255
        mask_r = cv2.inRange(hsv_image, red_lower, red_upper)

        # Color GREEN
        green_lower = np.array([49, 39, 130], np.uint8)  # 25,52,72
        green_upper = np.array([98, 255, 255], np.uint8)  # 102,255,255
        mask_g = cv2.inRange(hsv_image, green_lower, green_upper)

        mix_mask = mask_r + mask_g

        image_g = cv2.bitwise_and(cv_image, cv_image, mask=mask_g)
        image_r = cv2.bitwise_and(cv_image, cv_image, mask=mask_r)
        image = cv2.add(image_g, image_r)

        if np.array_equal(mix_mask, mask_g):
            self.vel.linear.x = 0.5
            print("Green color detected!")

        elif np.array_equal(mix_mask, mask_r):
            self.vel.linear.x = 0.0
            print("Red color detected!")

        else:
            self.vel.linear.x = 0.0
            print("None valid color detected!")

        cv2.imshow("Detect Color", image)
        cv2.waitKey(1)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
            self.pub_twist.publish(self.vel)
        except CvBridgeError as e:
            print(e)


def main(args):
    ic = image_converter()
    rospy.init_node("image_converter", anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
