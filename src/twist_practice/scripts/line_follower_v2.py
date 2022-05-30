#!/usr/bin/env python

import cv2
import numpy
import rospy
import cv_bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo


class LineFollower:
    def __init__(self):
        # Comportamiento para cuando se interrumpa el programa
        rospy.on_shutdown(self._cleanup)
        # Se crea el objeto cvBridge para traer la imagen del topic
        self.bridge = cv_bridge.CvBridge()
        # Subscriber
        self.image_sub = rospy.Subscriber(
            "/camera/image_raw", Image, self.image_callback
        )
        # Publisher
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # Variable twist para controlar la velocidad del robot
        self.twist = Twist()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Valores RGB para la máscara del color (negro en el caso de la pista)
        lower_color = numpy.array([0, 0, 0])
        upper_color = numpy.array([0, 0, 0])

        # Se crea la máscara usando los límites
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Altura, ancho y profundidad de la imagen (sólo se usan altura y ancho)
        h, w, d = image.shape

        # Región de la máscara donde el bot estará buscando la línea (dentro de la altura)
        # Parte superior
        search_top = 3*h//4
        # Parte inferior
        search_bot = 3*h//4+h//12
        # Parte izquierdas
        search_left = w//3
        # Parte derecha
        search_right = w//3+w//3 

        # Se limpia la imagen
        # (se eliminan los pixeles fuera del límite de search_top y bottom)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        
        mask[0:h, 0:search_left] = 0
        mask[0:h, search_right:w] = 0

        # Se obtienen los momentos de la máscara (https://theailearner.com/tag/cv2-moments/)
        M = cv2.moments(mask)
        # M00 es el área de la máscara (si el área es mayor a 0...)
        if M["m00"] > 0:
            # Centroide en 'x' y 'y' de la máscara (fórmulas en la liga anterior)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            """
            Control del robot:
            El error es igual a la diferencia entre el centro en 'x' de 
            la figura actual (imagen de la cámara) y la mitad del ancho 
            de la pantalla. La velocidad en 'x' del robot es constante (0.2).
            La velocidad angular dependerá de cuánto deba compensar el robot; 
            entre mayor sea el error, mayor será su velocidad angular.
            """
            
            err = cx - w / 2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -err / 95
                
            # Se publica el Twist del robot
            self.cmd_vel_pub.publish(self.twist)

            # Se despliega lo que ve el puzzlebot
            cv2.imshow("mask", mask)
            cv2.imshow("output", image)
            cv2.waitKey(3)

    def _cleanup(self):
        # Detiene el robot
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)


if __name__ == "__main__":
    rospy.init_node("follower")
    LineFollower()
    rospy.spin()
