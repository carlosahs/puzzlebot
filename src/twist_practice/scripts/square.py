#!/usr/bin/env python3

import rospy 
import math
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32 

#This class will make the puzzlebot move following a square.
class SquareClass(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 
        
        ###******* INIT PUBLISHERS *******### 
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        ############ CONSTANTS ################ 
        r = 0.05  #wheel radius (m)
        L = 0.18  #wheel separation (m)
        d = 0.0   #distance (m)
        theta = 0 #angle
        
        self.wr = 0.0  #Float32
        self.wl = 0.0  #Float32
        
        self.vel = Twist() #Robot's speed
        
        # Help variables
        final_d = 1.0
        final_a = math.pi/2
        restart = 0.0
        
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb) 
        
        #********** INIT NODE **********### 
        freq = 20
        rate = rospy.Rate(freq) #freqHz 
        dt = 1/freq #the time between one calculation and the next one
        
        print("Node initialized 10hz")
        while not rospy.is_shutdown(): 
            v = r*(self.wr + self.wl)/2
            w = r*(self.wr - self.wl)/L
            d = v*dt+d
            theta = w*dt+theta
            # print(f"The distance is: {d} m")
            # print(f"The velocity is: {v} m/s")
            # print(f"The angle is {theta}")
            if d >= final_d:
                self.stop()
                if theta < final_a:
                    self.left()       
                else:
                    d = restart
                    theta = restart       
            else:
                self.forward()
            
            self.pub.publish(self.vel) #publish the Robot's speed
            rate.sleep() 
        
    def wl_cb(self, wl): 
        self.wl = wl.data
        
    def wr_cb(self, wr): 
        self.wr = wr.data
        
    def stop(self): 
    # Function used it to stop puzzlebot
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        
    def forward(self):
    # Function used it to make the puzzlebot move forward
        self.vel.linear.x = 0.5
        self.vel.angular.z = 0
        
    def left(self):  
    # Function used it to make the puzzlebot turn left
        self.vel.angular.z = 0.5
        
    def cleanup(self): 
        self.wr = 0.0
        self.wl = 0.0
        self.stop()
        print('Bye bye!')
 

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("square", anonymous=True) 
    SquareClass() 
    
    
