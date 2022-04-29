#!/usr/bin/env python3
from __future__ import print_function

import roslib
import numpy as np
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from skimage.color import rgb2gray, gray2rgb


class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("video_source/raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)

        # (rows,cols,channels) = cv_image.shape
        # if cols > 60 and rows > 60 :
        #  cv2.circle(cv_image, (50,50), 10, 255)
        gray_image = rgb2gray(cv_image)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Color RED
        red_lower = np.array([0, 88, 179], np.uint8)
        red_upper = np.array([33, 255, 255], np.uint8)

        mask_r = cv2.inRange(hsv_image, red_lower, red_upper)
        red = cv2.bitwise_and(hsv_image, hsv_image, mask=mask_r)

        cv2.imshow("Image window", red)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(mask_r))
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
