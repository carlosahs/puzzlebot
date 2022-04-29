#!/usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan


class ClosestDetectorClass:
    """
    This class receives a LaserScan and finds the closest object
    """

    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        # Subscribers
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb)

        # Init node
        r = rospy.Rate(1)  # 1Hz

        self.closest_range = 0.0
        self.closest_angle = 0.0

        print("Node initialized 1hz")

        while not rospy.is_shutdown():
            range = self.closest_range
            theta = self.closest_angle
            theta = np.arctan2(np.sin(theta), np.cos(theta))

            r.sleep()

    def laser_cb(self, msg):
        """
        This function receives a number from hls lidar
        """
        closest_range = min(msg.ranges)
        idx = msg.ranges.index(closest_range)
        closest_angle = msg.angle_min + idx * msg.angle_increment

        print("closest object distance: " + str(closest_range))
        print("closest object direction: " + str(closest_angle))

        self.closest_angle = closest_angle
        self.closest_range = closest_range

    def cleanup(self):
        """
        This function is called just before finishing the node
        You can use it to clean things up before leaving
        Example: stop the robot before finishing a node.
        """
        pass


# Main program
if __name__ == "__main__":
    rospy.init_node("closest_detector", anonymous=True)
    ClosestDetectorClass()
