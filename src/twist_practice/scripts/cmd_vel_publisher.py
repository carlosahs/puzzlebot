#!/usr/bin/env python3 

#Se importa la librería rospy
import rospy   

#Se importa Twist de la librería geometry_msgs
from geometry_msgs.msg import Twist 

if __name__ =="__main__":
    
    # Se inicializa el nodo anónimo 'cmd_vel_publisher' 
    rospy.init_node('cmd_vel_publisher', anonymous=True) 
    
    # Se crea el publisher 'cmd_vel' de tipo de dato Twist
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
    
    # Refresh rate de 10hz
    rate = rospy.Rate(10)
    
    # Se crea la variable a publicar 'robot_vel' de tipo Twist
    robot_vel = Twist()
    
    # Se cambia el componente x de la velocidad linear del robot
    robot_vel.linear.x = 0.1
    
    # Ciclo while que seguirá hasta que se mate el proceso
    while not rospy.is_shutdown(): 
        # Publica el twist del robot
        pub.publish(robot_vel) 
        # Espera para volver a publicar el mensaje
        rate.sleep() 
