#!/usr/bin/env python

import time
import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class LiDAR:
    def __init__(self):
        self.lidar_msg = None
        rospy.Subscriber("base_scan", LaserScan, self._callback)

    # def start(self):
    #     pass

    def available(self):
        return self.lidar_msg is not None

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

    def get_ranges(self):
        return self.lidar_msg.ranges

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
    WHEEL_RADIUS = 0.05  # Wheel radius (m)
    WHEEL_SEPARATION = 0.19  # Wheel separation (m)

    def __init__(self):
        rospy.Subscriber("wl", Float32, self._wl_callback)
        rospy.Subscriber("wr", Float32, self._wr_callback)

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pos_pub = rospy.Publisher("pos", String, queue_size=1)

        self.vel = Twist()

        self.x = 0.0
        self.y = 0.0
        self.w = 0.0

        self.wl = None
        self.wr = None

    def goto_point(self, x, y, dt, clock):
        # Do nothing if nodes are not available
        if self.wl is None and self.wr is None:
            return

        # Control constants
        KV = 0.3
        KW = 0.5
        THRESHOLD = 0.1

        w_err = 2 * THRESHOLD
        d_err = 2 * THRESHOLD

        # print(str(self.x) + " " + str(self.y))

        at_point = False

        while not at_point:
            print(str(w_err) + " " + str(d_err))

            self.w = (
                self.WHEEL_RADIUS * (self.wr - self.wl)
                / self.WHEEL_SEPARATION * dt + self.w
            )

            # Limit angle range to [-pi, pi]
            self.w = np.arctan2(np.sin(self.w), np.cos(self.w))

            self.x = (
                self.x + self.WHEEL_RADIUS * (self.wr + self.wl)
                / 2 * dt * np.cos(self.w)
            )
            self.y = (
                self.y + self.WHEEL_RADIUS * (self.wr + self.wl)
                / 2 * dt * np.sin(self.w)
            )

            w_err = np.arctan2(y, x) - self.w
            d_err = np.sqrt((x - self.x) ** 2 + (y - self.y) ** 2)

            self.set_linear_vel(0.0)
            self.set_angular_vel(0.0)

            if abs(w_err) >= THRESHOLD:
                self.set_linear_vel(0.0)
                self.set_angular_vel(KW * w_err)
            elif abs(d_err) >= THRESHOLD:
                self.set_linear_vel(KV * d_err)
                self.set_angular_vel(0.0)
            else:
                self.set_linear_vel(0.0)
                self.set_angular_vel(0.0)

                at_point = True

            self.pub_vel()
            clock.sleep()

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_w(self):
        return self.w

    def pub_pos(self):
        self.pos_pub.publish(
            "(" + str(self.x) + ", " + str(self.y) + ") " + str(self.w)
        )

    def pub_vel(self):
        self.vel_pub.publish(self.vel)

    def set_linear_vel(self, vel):
        self.vel.linear.x = vel

    def set_angular_vel(self, vel):
        self.vel.angular.z = vel

    def cleanup(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0

        self.x = 0.0
        self.y = 0.0

        self.wl = None
        self.wr = None

        self.pub_vel()

    def _wl_callback(self, wl):
        self.wl = wl.data

    def _wr_callback(self, wr):
        self.wr = wr.data


class Main:
    KW = 0.3  # Angular speed gain
    KV_MAX = 0.3  # Maximum linear speed gain
    GROWTH_RATE = 1.0  # Exponential growth rate

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

    def start(self):
        freq = 10.0
        rate = rospy.Rate(freq)

        xt, yt = 0.0, 0.0

        while not rospy.is_shutdown():
            xt = float(input("x: "))
            yt = float(input("y: "))

            self.robot.goto_point(xt, yt, 1.0 / freq, rate)
            # if self.lidar.available():
            #     if np.isinf(self.lidar.get_min_range()):
            #         self.robot.set_linear_vel(0.0)
            #         self.robot.set_angular_vel(0.0)
            #     else:
            #         self.control_speed()
            rate.sleep()

    def control_speed(self):
        thetaT, dT = self.lidar.get_thetaT_dT()

        kv = self.KV_MAX * (
            1 - np.exp(-self.GROWTH_RATE * dT ** 2)
        ) / dT
        
        self.robot.set_linear_vel(kv * dT)
        self.robot.set_angular_vel(self.KW * thetaT)

    def cleanup(self):
        self.lidar.cleanup()
        self.robot.cleanup()


if __name__ == "__main__":
    rospy.init_node("avoid_obstacle", anonymous=True)
    main = Main()
    main.start()
