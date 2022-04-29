#!/usr/bin/env python3

# Se importa la librería rospy para poder usar ros con python
import rospy 

# Se importa String de la librería std_msgs para poder manejar los comandos recibidos
from std_msgs.msg import String 

#Se importa Twist de la librería geometry_msgs para controlar el twist del robot
from geometry_msgs.msg import Twist 

# Esta clase recibirá un comando en string y lo traducirá a cambios en el twist
# del robot de manera recursiva.
class ControlClass(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 
        
        # Inicializar subscribers
        rospy.Subscriber("twist", Twist, self.twist_cb) 
        rospy.Subscriber("string_command", String, self.cmd_cb)

        # Inicializar publisher
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        
        # Variables
        self.twist = Twist()
        self.cmd = String
        
        # Refresh rate (2 Hz)
        r = rospy.Rate(2) 
        
        # Log counter
        n = 1
        print("\nNode initialized 2 Hz\n")
        
        # Loop a ejecutar mientras el script esté corriendo
        while not rospy.is_shutdown(): 
            # Interpretar el comando recibido
            if(self.cmd.data == 'forward'):
                self.twist = Twist()
                self.twist.linear.x = 0.1
            elif(self.cmd.data == 'reverse'):
                self.twist = Twist()
                self.twist.linear.x = -0.1
            elif(self.cmd.data == 'left'):
                self.twist = Twist()
                self.twist.angular.z = 0.5
            elif(self.cmd.data == 'right'):
                self.twist = Twist()
                self.twist.angular.z = -0.5
            elif(self.cmd.data == 'stop'):
                self.twist = Twist()
            elif(self.cmd.data == 'donut'):
                self.twist.linear.x = 0.3
                self.twist.angular.z = 0.5
            # Publicar el twist del robot
            self.pub_vel.publish(self.twist)
            
            # Imprime la información del twist en la consola
            print("Log: " + str(n))
            print("Command: " + str(self.cmd.data))
            print(str(self.twist) + '\n')
            
            # Espera medio segundo
            r.sleep() 
            # Aumenta el log counter
            n = n + 1

    # Lee el twist recibido del subscriber
    def twist_cb(self, twist): 
        self.twist = twist
        pass 
        
    # Lee el string_command recibido del subscriber    
    def cmd_cb(self, cmd): 
        self.cmd = cmd
        pass 
        
    # Funciones a ejecutar antes de que el nodo se detenga
    def cleanup(self): 
        # Todas las velocidades a 0 (Detiene el robot)
        self.twist = Twist()
        self.pub_vel.publish(self.twist)
        print('\nProcess Terminated. Stopping Robot...')
        
        pass 

if __name__ == "__main__": 
    # Inicializa el nodo
    rospy.init_node("aaron_robot_control", anonymous=True) 
    
    #Llama la clase de control del robot
    ControlClass() 
    
    
