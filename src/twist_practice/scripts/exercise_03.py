#!/usr/bin/env python
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# This class will receive a ROS image and transform it to opencv format


class ColorFilter:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        ############ CONSTANTS ################
        self.bridge_object = CvBridge()  # create the cv_bridge object
        self.image_received = (
            0  # Flag to indicate that we have already received an image
        )

        ############################### SUBSCRIBERS #####################################
        image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_cb)

        # ********** INIT NODE **********###

        r = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            image = cv2.imread(
                "/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_2/Course_images/Filtering.png"
            )
            # I resized the image so it can be easier to work with
            image = cv2.resize(image, (300, 300))
            # Once we read the image we need to change the color space to HSV
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            # Hsv limits are defined
            # here is where you define the range of the color youre looking for
            # each value of the vector corresponds to the H,S & V values respectively

            min_green = np.array([50, 220, 220])
            max_green = np.array([60, 255, 255])
            min_red = np.array([170, 220, 220])
            max_red = np.array([180, 255, 255])
            min_blue = np.array([110, 220, 220])
            max_blue = np.array([120, 255, 255])

            # This is the actual color detection
            # Here we will create a mask that contains only the colors defined in your limits
            # This mask has only one dimension, so its black and white }

            mask_g = cv2.inRange(hsv, min_green, max_green)
            mask_r = cv2.inRange(hsv, min_red, max_red)
            mask_b = cv2.inRange(hsv, min_blue, max_blue)

            # We use the mask with the original image to get the colored post-processed image
            res_b = cv2.bitwise_and(image, image, mask=mask_b)
            res_g = cv2.bitwise_and(image, image, mask=mask_g)
            res_r = cv2.bitwise_and(image, image, mask=mask_r)

            cv2.imshow("Original", image)
            cv2.imshow("Green", res_g)
            cv2.imshow("Red", res_r)
            cv2.imshow("Blue", res_b)

            cv2.waitKey(1)

            r.sleep()

    def image_cb(self, ros_image):
        """
        This function receives a ROS image and transforms it into opencv format
        """
        try:
            print("received ROS image, I will convert it to opencv")
            # We select bgr8 because its the OpenCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(
                ros_image, desired_encoding="bgr8"
            )

            self.image_received = 1  # Turn the flag on
        except CvBridgeError as e:
            print(e)

    def cleanup(self):
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("color_filter", anonymous=True)
    ColorFilter()
