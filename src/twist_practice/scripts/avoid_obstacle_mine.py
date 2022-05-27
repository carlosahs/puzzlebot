#!/usr/bin/env python3

import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class LiDAR:
    def __init__(self):
        rospy.Subscriber("base_scan", LaserScan, self._callback)
        self.lidar_msg = None

    # def start(self):
    #     pass

    def available(self):
        return self.lidar_msg is None

    def get_x_y(self, r, angle):
        return self._polar_to_cartesian(r, angle)

    def get_xT_yT(self):
        xT, yT = 0, 0

        for x, y in self._generate_xs_ys():
            xT += x
            yT += y

        return xT, yT

    def get_thetaT_dT(self):
        xT, yT = self.get_xT_yT()

        thetaT = np.arctan2(yT, xT)
        dT = np.sqrt(xT ** 2 + yT ** 2)

        return thetaT, dT

    def get_range(self, idx):
        return self.lidar_msg.ranges[idx]

    def get_angle(self, idx):
        angle = self.lidar_msg.angle_min + idx * self.lidar_msg.angle_increment
        angle = np.arctan2(np.sin(angle), np.cos(angle))

        return angle

    def get_min_range(self):
        return min(self.lidar_msg.ranges)

    def get_min_angle(self):
        min_range = self.get_min_range()
        min_range_idx = self.lidar_msg.ranges.index(min_range)

        min_angle = (
            self.lidar_msg.angle_min + min_range_idx * self.lidar_msg.angle_increment
        )
        min_angle = np.arctan2(np.sin(min_angle), np.cos(min_angle))

        return min_angle

    def cleanup(self):
        self.lidar_msg = None

    def _callback(self, msg):
        self.lidar_msg = msg

    def _polar_to_cartesian(self, r, angle):
        if np.isinf(r):
            r = self.lidar_msg.range_max

        x = r * np.cos(angle)
        y = r * np.sin(angle)

        return x, y

    def _generate_xs_ys(self):
        for i in range(len(self.lidar_msg.ranges)):
            angle = self.get_angle(i)
            r = self.get_range(i)

            x, y = self._polar_to_cartesian(r, angle)

            yield (x, y)


class Robot:
    def __init__(self):
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.vel = Twist()

    def pub_vel(self):
        self.vel_pub.publish(self.vel)

    def set_linear_vel(self, vel):
        self.vel.linear.x = vel

    def set_angular_vel(self, vel):
        self.vel.angular.z = vel

    def cleanup(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0

        self.pub_vel(self.vel)


class Main:
    def __init__(self):
        self.lidar = LiDAR()
        self.robot = Robot()

        rospy.on_shutdown(self.cleanup)

        if self.lidar.available():
            self.min_angle = self.lidar.get_min_angle()
            self.min_range = self.lidar.get_min_range()
        else:
            self.min_angle = 0.0
            self.min_range = np.inf

        rate = rospy.Rate(10)

    def cleanup(self):
        self.lidar.cleanup()
        self.robot.cleanup()


class AvoidObstacle:
    def __init__(self):
        rospy.on_shutdown(self._cleanup)

        # PUBLISHERS AND SUBSCRIBERS
        rospy.Subscriber("base_scan", LaserScan, self._laser_cb)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # VARIABLES
        self.closest_angle = 0.0  # Angle to the closest object
        self.closest_range = np.inf  # Distance to the closest object

        vel_msg = Twist()
        r = rospy.Rate(10)  # 10Hz is the lidar's frequency
        print("Node initialized 10hz")

        # LiDAR info
        self.lidar_msg = None
        self.x_from_lidar = None
        self.y_from_lidar = None

        # MAIN LOOP
        while not rospy.is_shutdown():
            range = self.closest_range
            theta_closest = self.closest_angle
            thetaA0 = theta_closest - np.pi
            thetaA0 = np.arctan2(np.sin(thetaA0), np.cos(thetaA0))
            # print("closest object distance: " + str(self.closest_range))
            # print("theta_closest: " + str(theta_closest))

            self._lidar_to_xy()

            # Activity 1: Print corresponding x and y point from angle and distance
            x, y = self._xy_from_tr(theta_closest, self.closest_range)
            # print(f"x from closest: {x}")
            # print(f"y from closest: {y}")

            # Activity 2
            if self.x_from_lidar is not None and self.y_from_lidar is not None:
                # Print x and y points
                # print(f"x points: {self.x_from_lidar}")
                # print(f"y points: {self.y_from_lidar}")

                # Get Xt and Yt
                Xt, Yt = self._get_Xt_Yt()
                # print(f"Xt: {Xt}")
                # print(f"Yt: {Yt}")

                # Theta T and distance T
                thetaT, distanceT = self._get_thetaT_distanceT(Xt, Yt)
                # print(f"Theta T: {thetaT}")
                # print(f"Distance T: {distanceT}")
                time.sleep(2)

                # # Activity 3: Robot's angular and linear velocities
                v = self.V_DESIRED * distanceT
                w = self.KW * thetaT

                if np.isposinf(range):  # no obstacles condition
                    vel_msg.linear.x = v
                    vel_msg.angular.z = 0.0
                elif range <= self.MIN_OBSTACLE_RANGE:
                    vel_msg.linear.x = 0.0
                    vel_msg.angular.z = 0.0
                else:
                    vel_msg.linear.x = v
                    vel_msg.angular.z = w

            self.cmd_vel_pub.publish(vel_msg)
            r.sleep()

    def _xy_from_tr(self, angle, distance):
        """
        Compute x and y from angle and distance
        """
        return distance * np.cos(angle), distance * np.sin(angle)

    def _get_Xt_Yt(self):
        """
        Compute xt and yt
        """
        xt = (np.clip(self.x_from_lidar, -8.0, 8.0)).sum()
        yt = (np.clip(self.y_from_lidar, -8.0, 8.0)).sum()
        return xt, yt

    def _get_thetaT_distanceT(self, xt, yt):
        """
        Compute theta t and distance try:
        """
        tt = np.arctan2(yt, xt)
        dt = np.sqrt(xt**2 + yt**2)
        return tt, dt

    def _lidar_to_xy(self):
        if self.lidar_msg is None:  # no LiDAR object instantiated
            return

        angle_increment = self.lidar_msg.angle_increment
        angle_min = self.lidar_msg.angle_min

        distance = [range for range in self.lidar_msg.ranges]
        angle = [angle_min + i * angle_increment for i in range(len(distance))]

        self.x_from_lidar = np.array([r * np.cos(t) for r, t in zip(distance, angle)])
        self.y_from_lidar = np.array([r * np.sin(t) for r, t in zip(distance, angle)])

    def _laser_cb(self, msg):
        """
        This function receives a message of type LaserScan and computes the
        closest object direction and range
        """
        self.lidar_msg = msg
        closest_range = min(self.lidar_msg.ranges)
        idx = self.lidar_msg.ranges.index(closest_range)
        closest_angle = self.lidar_msg.angle_min + idx * self.lidar_msg.angle_increment
        # Limit the angle to [-pi,pi]
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle))
        self.closest_range = closest_range
        self.closest_angle = closest_angle

    def _cleanup(self):
        """
        This function is called just before finishing the node
        You can use it to clean things up before leaving
        Example: stop the robot before finishing a node.
        """
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)


if __name__ == "__main__":
    rospy.init_node("avoid_obstacle", anonymous=True)
    AvoidObstacle()
