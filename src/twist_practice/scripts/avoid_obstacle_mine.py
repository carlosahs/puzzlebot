#!/usr/bin/env python3

import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class AvoidObstacle:
    def __init__(self):
        rospy.on_shutdown(self._cleanup)
        # PUBLISEHRS AND SUBSCRIBERS
        rospy.Subscriber("base_scan", LaserScan, self._laser_cb)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # CONSTANTS AND VARIABLES
        MIN_OBSTACLE_RANGE = 0.3  # robot's circumscribed radius
        v_desired = 0.00008  # speed when there are no obstacles
        kw = 0.3  # angular speed gain
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
            #print("closest object distance: " + str(self.closest_range))
            #print("theta_closest: " + str(theta_closest))

            self._lidar_to_xy()

            # Activity 1: Print corresponding x and y point from angle and distance
            x, y = self._xy_from_tr(theta_closest, self.closest_range)
            #print(f"x from closest: {x}")
            #print(f"y from closest: {y}")

            # Activity 2
            if self.x_from_lidar is not None and self.y_from_lidar is not None:
                # Print x and y points
                #print(f"x points: {self.x_from_lidar}")
                #print(f"y points: {self.y_from_lidar}")

                # Get Xt and Yt
                Xt, Yt = self._get_Xt_Yt()
                #print(f"Xt: {Xt}")
                #print(f"Yt: {Yt}")

                # Theta T and distance T
                thetaT, distanceT = self._get_thetaT_distanceT(Xt, Yt)
                #print(f"Theta T: {thetaT}")
                #print(f"Distance T: {distanceT}")
                time.sleep(2)
            
                # # Activity 3: Robot's angular and linear velocities
                v = v_desired * distanceT
                w = kw * thetaT
                if np.isposinf(range):  # no obstacles condition
                    vel_msg.linear.x = v
                    vel_msg.angular.z = 0.0
                elif range <= MIN_OBSTACLE_RANGE:
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
    
