#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import cv_bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
import time
import os

class Admin_machine:
    def __init__(self):
        # Comportamiento para cuando se interrumpa el programa
        rospy.on_shutdown(self._cleanup)

        # Subscribers
        self.line_sub = rospy.Subscriber("/line_vel", Twist, self.line_cb) #Suscriptor a la velocidad del nodo de line follower
        self.signal_sub = rospy.Subscriber("/signal_detected", Int32, self.signal_cb)
        self.sem_sub = rospy.Subscriber("/sem_color", Int32, self.sem_cb) #Suscriptor a la deteccion de los semaforos
        self.preds_sub = rospy.Susbcriber

        # Publishers
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Variable twist para controlar la velocidad del robot
        self.twist = Twist()
        self.line_vel = Twist()

        self.signal_detected = -1
        self.sem_color = 3
        counter = 0
        state = 'Follow_line'

        rate = 20
        r = rospy.Rate(rate)
        print("Node initialized at " + str(rate) + "Hz.")
        while not rospy.is_shutdown():
            signal = self.signal_detected
            sem = self.sem_color
            
            if state == 'Follow_line':
                if signal == 2:
                    state = 'Increase_speed'
                elif sem == 4:
                    state = 'Stop'
                elif signal == 0 and sem == 3:
                    counter = 0
                    state = 'Intersection'
                elif signal == 6:
                    state = 'Stop'
                else:
                    print('Following Line')
                    self.twist = self.line_vel
            elif state == 'Increase_speed':
                if sem == 4:
                    state = 'Stop'
                elif signal == 1 and sem == 3:
                    counter = 0
                    state = 'Turn_right'
                else:
                    print('Going faster')
                    self.twist = self.line_vel
                    self.twist.linear.x = 0.18

            elif state == 'Turn_right':
                if counter > 70:
                    state = 'Follow_line'
                else:
                    print('Turnning right')
                    print('Counter: ', counter)
                    self.twist.angular.z = -0.1
                    self.twist.linear.x = 0.15
                    counter += 1

            elif state == 'Intersection':
                if counter > 60:
                    state = 'Follow_line'
                else:
                    print('Crossing intersection')
                    print('Counter: ', counter)
                    self.twist.angular.z = 0.0
                    self.twist.linear.x = 0.13
                    counter += 1
            
            elif state == 'Stop':
                if signal == 1 and sem == 3:
                    counter = 0
                    state = 'Turn_right'
                elif signal == 0 and sem == 3:
                    counter = 0
                    state = 'Intersection'
                else:
                    print('Stopped')
                    self.twist.angular.z = 0.0
                    self.twist.linear.x = 0.0
            
            else:
                print('No state defined')
                self.twist.angular.z = 0.0
                self.twist.linear.x = 0.0
            

            self.cmd_vel_pub.publish(self.twist)    #Publicamos la velocidad al puzzlebot
            r.sleep()

    # def line_cb(self, line_msg):
    #     self.line_vel = line_msg

    def signal_cb(self, signal_msg):
        self.signal_detected = signal_msg.data

    def sem_cb(self, sem_msg):
        self.sem_color = sem_msg.data
    
    def _cleanup(self):
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)

if __name__ == "__main__":
    rospy.init_node("follower", anonymous=True)
    Admin_machine()
    rospy.spin()
