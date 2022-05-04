#!/usr/bin/env python3
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ShowImage:
    def __init__(self):
        # Constants
        self.bridge_object = CvBridge()  # create the cv_bridge object

        self.image_received = (
            0  # Flag to indicate that we have already received an image
        )

        # Subscribers
        image_sub = rospy.Subscriber("image_topic", Image, self.image_cb)

        # Init node
        r = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            if self.image_received:
                print("Do something with the image")

            cv2.waitKey(1)
            r.sleep()

        cv2.destroyAllWindows()

    def image_cb(self, ros_image):
        """
        This function receives a ROS image and transforms it into opencv format
        """
        try:
            print("received ROS image, I will convert it to opencv")

            # We select bgr8 because it is the OpenCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(
                ros_image, desired_encoding="bgr8"
            )
            self.image_received = 1  # Turn the flag on
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    rospy.init_node("cv_bridge_example", anonymous=True)
    ShowImage()
