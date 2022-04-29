#!/usr/bin/env python3
import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ClosestDetector:
    KMAX = 0.9
    ALPHA = 1.0
    STOP_RANGE = 0.5
    KW = 1.0  # Angular velocity gain

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        # Subscribers
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        vel_msg = Twist()

        # kv = 0.6 # Constant to change the linear speed
        # kw = 1.0  # Angular velocity gain

        self.closest_range = 0.0  # Distance to the closest object
        self.closest_angle = 0.0  # Angle to the closest object

        # Init node
        r = rospy.Rate(1)  # 10Hz is the lidar's frequency
        print("Node initialized 1hz")

        while not rospy.is_shutdown():
            range = self.closest_range
            theta = np.arctan2(
                np.sin(self.closest_angle),
                np.cos(self.closest_angle)
            )

            # Limit the angle to -pi < theta < pi
            if (range <= self.STOP_RANGE):
                kv = 0.0
            else:
                kv = self._gain_adj(range)

            if np.isposinf(range):
                print("No object detected")
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
            else:
                vel_msg.linear.x = kv * range
                vel_msg.angular.z = self.KW * theta

            self.cmd_vel_pub.publish(vel_msg)

            print("vel_msg.linear.x: " + str(vel_msg.linear.x))
            print("vel_msg.angular.z: " + str(vel_msg.angular.z))

            r.sleep()

    def _gain_adj(self, range):
        """
        Adjust gain K considering it as a function of the error
        """
        return self.KMAX * (
            (1.0 - math.exp(-self.ALPHA * abs(range) ** 2))
            / abs(range)
        )

    def laser_cb(self, msg):
        """
        This function receives a number
        For this lidar
        """
        self.closest_range = min(msg.ranges)
        idx = msg.ranges.index(self.closest_range)
        self.closest_angle = msg.angle_min + idx * msg.angle_increment

        print("Closest object distance: " + str(self.closest_range))
        print("Closest object direction: " + str(self.closest_angle))

    def cleanup(self):
        """
        This function is called just before finishing the node
        You can use it to clean things up before leaving
        Example: stop the robot before finishing a node.
        """
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)


# Main program
if __name__ == "__main__":
    rospy.init_node("closest_detector", anonymous=True)
    ClosestDetector()
