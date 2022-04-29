#!/usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import String


class position:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        ###******* INIT PUBLISHERS *******###
        self.pub_pos = rospy.Publisher("pos", String, queue_size=1)
        self.pub_twist = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        ############ CONSTANTS ################
        r = 0.05  # wheel radius (m)
        L = 0.19  # wheel separation (m)
        theta = 0  # angle
        x = 0  # position in x
        y = 0  # position in y

        self.des_x = 0.0  # Float32
        self.des_y = 0.0  # Float32
        self.wr = 0.0  # Float32
        self.wl = 0.0  # Float32
        self.vel = Twist()

        xt = 0.0
        yt = 0.0

        ############################### SUBSCRIBERS #####################################
        rospy.Subscriber("wl", Float32, self.wl_cb)
        rospy.Subscriber("wr", Float32, self.wr_cb)

        # ********** INIT NODE **********###
        freq = 20.0
        rate = rospy.Rate(freq)  # freqHz
        dt = 1 / freq  # the time between one calculation and the next one

        print("Node initialized 20hz")
        while not rospy.is_shutdown():
            if xt == 0 and yt == 0:
                xt = float(input("x: "))
                yt = float(input("y: "))
                print("----------------------------")

            theta = round(r * (self.wr - self.wl) / L * dt + theta, 3)
            x = round(x + r * (self.wr + self.wl) / 2 * dt * math.cos(theta), 3)
            y = round(y + r * (self.wr + self.wl) / 2 * dt * math.sin(theta), 3)

            et = round(np.arctan2(yt, xt) - theta, 3)
            ed = round(math.sqrt((xt - x) ** 2 + (yt - y) ** 2), 3)

            if theta > math.pi:
                theta = theta - 2 * math.pi
            elif theta < -math.pi:
                theta = theta + 2 * math.pi
            position = f"({x}, {y}, {theta}) -> ({ed}, {et})"

            self.pub_pos.publish(position)  # publish the Robot's speed

            # Move to a set position -------------------------------------------------
            kv = 0.3
            kw = 0.5

            self.vel.linear.x = 0
            self.vel.angular.z = 0

            if et >= 0.1 or -0.1 >= et:
                self.vel.angular.z = kw * et  # w = kw*et

            elif ed >= 0.10:
                self.vel.linear.x = kv * ed  # v = kv*ed
                self.vel.angular.z = 0

            # elif self.vel.linear.x == 0 and self.vel.angular.z == 0:
            # xt = 0
            # yt = 0

            self.pub_twist.publish(self.vel)
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            rate.sleep()

    def wl_cb(self, wl):
        self.wl = wl.data

    def wr_cb(self, wr):
        self.wr = wr.data

    def des_x(self, des_x):
        self.des_x = des_x.data

    def des_y(self, des_y):
        self.des_y = des_y.data

    def cleanup(self):
        self.wr = 0.0
        self.wl = 0.0
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.pub_twist.publish(self.vel)
        print("Bye bye!")


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
    rospy.init_node("position_pzbot", anonymous=True)
    position()
