#!/usr/bin/env python
import numpy as np
import socket
import cv2
import rospy
import cv_bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
import time
import os

SVR_ADD = 1246
SVR_QS = 5

CLT_ADD = 1247
CLT_QS = 5

BYTE_STREAM = 1024

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((socket.gethostname(), SVR_ADD))
s.listen(SVR_QS)

class FollowLine:
    def __init__(self):
        capture = cv2.VideoCapture(0)
        rospy.on_shutdown(self._cleanup)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(
            "video_source/raw", Image, self.image_callback
        )
        self.cmd_vel_pub = rospy.Publisher("/signal_detected", Int32, queue_size=1)
        self.image_pub = rospy.Publisher("/sem_color", Int32, queue_size=1)

        self.image_recieved = np.zeros((0,0))
        self.signal = 0
        self.semaphore = 0
        
        rate = 10
        r = rospy.Rate(rate)
        print("Node initialized at " + str(rate) + "Hz.")
        
        # model = torch.hub.load('ultralytics/yolov5', 'custom', path=MODELS + "/best_5s.pt")

        while not rospy.is_shutdown():
            ret, image = capture.read()
            # image = self.image_recieved
            print(image, image.nbytes, image.dtype, image.shape)

            if image.any() > 0:
            #     buf = image.tobytes()
            #     arr = np.frombuffer(buf, dtype=np.uint8)
            #
            #     print(arr.nbytes)

                clientsocket, address = s.accept()
                clientsocket.sendall(image.tobytes())
                clientsocket.close()

                c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                c.connect((socket.gethostname(), CLT_ADD))

                pred = c.recv(BYTE_STREAM)
                pred = int(pred.decode("utf-8"))

                print(pred)
            r.sleep()  
            
        image.release()


    def image_callback(self, msg):
        try:
            self.image_recieved = self.bridge.imgmsg_to_cv2(msg)
        except cv_bridge.CvBridgeError as e:
            print(e)
            
    def _cleanup(self):
        pass
    
if __name__ == "__main__":
    rospy.init_node("follower", anonymous=True)
    FollowLine()
    rospy.spin()
