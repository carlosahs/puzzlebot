#!/usr/bin/env python3
import sys

import numpy as np
import cv2
import rospy

from typing import List
from typing import Tuple

from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge, CvBridgeError
from skimage.color import rgb2gray, gray2rgb


class Puzzlebot:
    R = 0.05  # wheel radius
    L = 0.18  # wheel separation

    KV = 0.3  # Linear velocity gain
    KW = 0.5  # Angular velocity gain

    def __init__(self) -> None:
        rospy.on_shutdown(self._cleanup)

        self.twist = Twist()  # puzzlebot's twist

        # Puzzlebot coordinates
        self.x = 0.0
        self.y = 0.0

        self.x_err = 0.0
        self.y_err = 0.0

        # Puzzlebot's rotation
        self.rotation = 0.0
        self.rotation_err = 0.0

        # Angular velocity from left and right wheels
        self.wl = 0.0
        self.wr = 0.0

        rospy.Subscriber("wl", Float32, self._update_wl)
        rospy.Subscriber("wr", Float32, self._update_wr)

        self.position_pub = rospy.Publisher("puzzlebot_position", String, queue_size=1)
        self.twist_pub = rospy.Publisher("puzzlebot_twist", Twist, queue_size=1)

    def goto(self, position: Tuple[float, float], dt: float, threshold: float) -> None:
        x_target = position[0]
        y_target = position[1]

        self.x_err = x_target - self.x
        self.y_err = y_target - self.y

        self.update_coordinates(dt)

        self.rotation_err = np.arctan2(y_target, x_target) - self.rotation
        position_err = np.sqrt(self.x_err ** 2 + self.y_err ** 2)

        self.publish_position()

        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        if abs(position_err) >= threshold:
            self.twist.linear.x = self.KV * position_err
            self.twist.angular.z = 0.0
        elif abs(self.rotation_err) >= threshold:
            self.twist.angular.z = self.KW * self.rotation_err

        self.publish_twist()

        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

    def publish_twist(self) -> None:
        self.twist_pub.publish(self.twist)

    def publish_position(self) -> None:
        position = f"({self.x}, {self.y}, {self.rotation}) -> ({self.x_err}, {self.y_err}, {self.rotation_err})"
        self.position_pub.publish(position)

    def update_coordinates(self, dt: float) -> None:
        self.update_rotation()

        self.x = self.x + self.R * (self.wr + self.wl) / 2 * dt * np.cos(self.rotation)
        self.y = self.y + self.R * (self.wr + self.wl) / 2 * dt * np.sin(self.rotation)

    def update_rotation(self, dt: float) -> None:
        self.rotation = self.R * (self.wr - self.wl) / self.L * dt * self.rotation

        if self.rotation > np.pi:
            self.rotation -= 2 * np.pi
        elif self.rotation < -np.pi:
            self.rotation += 2 * np.pi

    def get_coordinates(self) -> Tuple[float, float]:
        return (self.x, self.y)

    def get_rotation(self) -> float:
        return self.rotation

    def get_wheel_radius(self) -> float:
        return self.R

    def get_wheel_separation(self) -> float:
        return self.L

    def _update_wr(self, wr: float) -> None:
        self.wr = wr.data

    def _update_wl(self, wl: float) -> None:
        self.wr = wl.data

    def _cleanup(self):
        self.wr = 0.0
        self.wl = 0.0

        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        self.pub_twist.publish(self.twist)
        print("Bye bye!")


class PathRunner:
    def __init__(self, path: List[Tuple[int]]) -> None:
        self.path = path
        self.puzzlebot = Puzzlebot()


if __name__ == "__main__":
    pass
