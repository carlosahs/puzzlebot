#!/usr/bin/env python3
import rospy 
import numpy as np
import math
from sensor_msgs.msg import LaserScan  
from geometry_msgs.msg import Twist
#This class will receive a laserScan and finds the closest object
class ClosestDetectorClass(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup)  
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb) 
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        vel_msg = Twist()
        ############ CONSTANTS ################ 
        self.closest_range = 0.0 # Distance to the closest object
        self.closest_angle = 0.0 # Angle to the closest object
        kv = 0.3 #Constant to change the linear speed
        kw = 1.0 #Angular velocity gain
        kmax = 0.8
        alpha = 0.9
        #********** INIT NODE **********### 
        r = rospy.Rate(10) #10Hz is the lidar's frequency
        print("Node initialized 10hz")
        while not rospy.is_shutdown(): 
            range = self.closest_range
            theta = self.closest_angle
            theta = np.arctan2(np.sin(theta),np.cos(theta))
            if range <= 0.60:
                kv = 0
            else:
                kv = kmax*((1.0-math.e**(-alpha*(abs(range)**2.0)))/abs(range))
            if np.isposinf(range):
                print("No object detected")
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
            else:
                vel_msg.linear.x = kv * range
                vel_msg.angular.z = kw * theta
            self.cmd_vel_pub.publish(vel_msg)
            r.sleep() 
            
    def laser_cb(self, msg): 
        ## This function receives a number
        #For this lidar
        self.closest_range = min(msg.ranges)
        idx = msg.ranges.index(self.closest_range)
        self.closest_angle = msg.angle_min + idx * msg.angle_increment
        print("Closest object distance: " + str(self.closest_range))
        print("Closest object direction: " + str(self.closest_angle))
        
        
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.   
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("closest_detector", anonymous=True) 
    ClosestDetectorClass()
