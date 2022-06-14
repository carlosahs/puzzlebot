#!/usr/bin/env python
import ast
import numpy as np
import socket
import cv2
import heapq
import rospy
import cv_bridge
import time
import os

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32

SVR_ADD = 9002
SVR_QS = 5

CLT_ADD = 9003
CLT_QS = 5

BYTE_STREAM = 4096
IMG_PATH = "/home/carlosahs42/Documents/imgbin.npy"

SIGNAL_THRESHOLD_HI = 40000
SIGNAL_THRESHOLD_LO = 20000

SEM_THRESHOLD = 3500

# def cmp_heapify(data, cmp):
#     s = list(map(cmp_to_key(cmp), data))
#     heapq.heapify(s)
#     return s

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((socket.gethostname(), SVR_ADD))
s.listen(SVR_QS)

class ImageDetection:
    def __init__(self):
        # capture = cv2.VideoCapture(0)
        rospy.on_shutdown(self._cleanup)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(
            "video_source/raw", Image, self.image_callback
        )
        self.sem_pub = rospy.Publisher("/sem_color", Int32, queue_size=1)
        self.signal_pub = rospy.Publisher("/signal_detected", Int32, queue_size=1)

        self.image_recieved = np.zeros((0,0))
        self.signal = 0
        self.semaphore = 0
        
        rate = 10
        r = rospy.Rate(rate)
        print("Node initialized at " + str(rate) + "Hz.")
        
        # model = torch.hub.load('ultralytics/yolov5', 'custom', path=MODELS + "/best_5s.pt")

        signals, semaphores = [], []

        while not rospy.is_shutdown():
            # ret, image = capture.read()
            image = self.image_recieved

            if image.any() > 0:
                with open(IMG_PATH, "wb") as f:
                    np.save(f, image)

                clientsocket, address = s.accept()
                clientsocket.send(bytes(IMG_PATH.encode("utf-8")))
                clientsocket.close()

                c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                c.connect((socket.gethostname(), CLT_ADD))

                buf = c.recv(BYTE_STREAM)
                data_dict = buf.decode("utf-8")
                data = ast.literal_eval(data_dict)

                # print(data)
                # 40k threshold

                signals, semaphores = self._get_preds_pq(data)

                if len(signals) > 0:
                    # print(signals[0])

                    if len(signals) > 0 and (
                        -signals[0][0] <= SIGNAL_THRESHOLD_HI and -signals[0][0] >= SIGNAL_THRESHOLD_LO
                    ):
                        self.signal_pub.publish(signals[0][1])
                else:
                    self.signal_pub.publish(-1)

                if len(semaphores) > 0:
                    # print(semaphores[0])

                    if len(semaphores) > 0 and semaphores[0][0] >= SEM_THRESHOLD:
                        self.sem_pub.publish(semaphores[0][1]) # Publish semaphore class
                else:
                    self.sem_pub.publish(-1)

            r.sleep()  
            
        image.release()

    def _get_preds_pq(self, data):
        num_signals = len(data["class"])
        signal_list = []
        semaphore_list = []

        # for key, val in data.items():
        #     pass

        for i in range(num_signals):
            signal_map = {}

            for key in data.keys():
                datum = data[key][str(i)]
                signal_map[key] = datum

            # xmin = signal_map["xmin"]
            # ymin= signal_map["ymin"]
            # xmax = signal_map["xmax"]
            # ymax = signal_map["ymax"]
            confidence = signal_map["confidence"]
            name = signal_map["name"]
            name_id = signal_map["class"]
            area = signal_map["area"]
            # area = abs(xmin - xmax) * abs(ymin - ymax)

            if signal_map["name"].find("semaphore") >= 0:
                semaphore_list.append((area, name_id, name, confidence))
            else:
                signal_list.append((-area, name_id, name, confidence))

        heapq.heapify(signal_list)
        heapq.heapify(semaphore_list)

        return signal_list, semaphore_list

    def image_callback(self, msg):
        try:
            self.image_recieved = self.bridge.imgmsg_to_cv2(msg)
        except cv_bridge.CvBridgeError as e:
            print(e)
            
    def _cleanup(self):
        pass
    
if __name__ == "__main__":
    rospy.init_node("detection", anonymous=True)
    ImageDetection()
    rospy.spin()
