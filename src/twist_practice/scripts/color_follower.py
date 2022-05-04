#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist


class ColorFollower:
    """
    Receives radius and center of object detected through robot's camera and
    move towards such object
    """

    XM = 300
    KV = 1.0  # 0.5
    KW = 0.001796875  # 0.00359375

    def __init__(self):
        rospy.on_shutdown(self._cleanup)

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("radius", Int32, self._radius_callback)
        rospy.Subscriber("center", Point, self._center_callback)

        self.vel = Twist()

        self.ball_radius = 0
        self.ball_center = Point()

        r = rospy.Rate(1)

        d_threshold = 5.5
        w_threshold = 5.5

        while not rospy.is_shutdown():
            # >>> SOLUTION
            x_err = self.XM - self.ball_center.x
            w_err = self.XM - self.ball_radius

            if abs(x_err) >= d_threshold:
                self.vel.angular.z = self.KW * x_err
            elif abs(w_err) >= w_threshold and self.ball_radius > 0:
                self.vel.angular.z = 0.0
                self.vel.linear.x = self.KV * 1 / self.ball_radius
            # <<< SOLUTION

            # print(self.ball_radius)
            print(self.ball_center)

            self.cmd_vel_pub.publish(self.vel)
            # print(self.vel)
            r.sleep()

        # v = K.v * 1 / {rad}
        # w = K.w(x.m - x.c)

    def _radius_callback(self, radius):
        self.ball_radius = radius.data

    def _center_callback(self, center):
        self.ball_center = center

    def _cleanup(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0

        self.cmd_vel_pub.publish(self.vel)


if __name__ == "__main__":
    rospy.init_node("color_follower", anonymous=True)
    ColorFollower()
