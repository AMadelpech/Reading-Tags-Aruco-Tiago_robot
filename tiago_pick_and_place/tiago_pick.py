import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class TiagoPickPlace(Node): #ArucoDetector
    def __init__(self):
        super().__init__('tiago_pick_and_place')
        self.publisher = self.create_publisher(Twist, '/key_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info("Tiago Publisher node started")
        
        self.subscription = self.create_subscription(Image, '/head_front_camera/image', self.image_callback, 10)
        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.get_logger().info(f"ArUco Dict: {self.aruco_dict}")

        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.get_logger().info(f"ArUco Params: {self.aruco_params}")

        #print de démarrage du sbscriber caméra:
        self.get_logger().info("Tiago Subscriber camera node started")

        # Variables pour déplacement jusqu'à détection de code Aruco 
        self.is_rotating = False
        self.rotation_angle = 0.0
        self.rotation_increment = 0.1  # radians
    
    def image_callback(self, msg):
        print("titi")
        self.get_logger().info("Image reçue")  # Vérifier que l’image arrive bien
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame is None:
            self.get_logger().error("Erreur : Image reçue est None !")
            return  # Stoppe la fonction pour éviter le crash
        self.get_logger().info("bonne frame")
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.get_logger().info(f"Image shape: {frame.shape}") #vérifie que la frame est bien chargée
        
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if gray is None or gray.size == 0:
            self.get_logger().error("Erreur : Image en niveaux de gris est vide !")
            return
        self.get_logger().info("detection de marker") 
        
        if ids is not None:
            self.get_logger().info(f"Code Aruco détecté: {ids.flatten()}")
            # Dessin du marqueur sur l’image
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Calcul de la position du centre du marqueur
            cx, cy = np.mean(corners[0][0], axis=0)

            #distance approchée : 
            approx_distance = 1000 / (cy + 1)

            self.move_to_aruco(cx, frame.shape[1], approx_distance)
            self.is_rotating = False  # Arrêter la rotation si un code est détecté
        else:
            self.get_logger().info("Aucun code Aruco détecté, rotation du robot.")
            self.rotate_robot()
            
        
        # Affichage de l’image en temps réel
        cv2.imshow("Tiago Camera - Aruco Detection", frame)
        cv2.waitKey(1)
      
    #envoie du message de type Twist qui envoie des vitesses aux moteurs 
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5  # Vitesse linéaire en m/s
        msg.angular.z = 0.0  # Vitesse angulaire en rad/s
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
    
    #rotation sur lui même quand pas de détéection de code Aruco 
    def rotate_robot(self):
        if not self.is_rotating:
            self.is_rotating = True
            self.rotation_angle = 0.0

        twist = Twist()
        twist.angular.z = self.rotation_increment  # Rotation incrémentale

        self.rotation_angle += self.rotation_increment
        if self.rotation_angle >= 2 * np.pi:  # Tour complet sur lui même
            self.is_rotating = False
            self.move_forward_slightly()

        self.publisher.publish(twist)
        self.get_logger().info(f"Rotation: {twist.angular.z:.2f}")

    # si toujours pas de détection: le robot avance légèrement avant de recommencer à tourner.
    def move_forward_slightly(self):
        twist = Twist()
        twist.linear.x = 0.1  # Avancer légèrement
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Avance légèrement")
    
    def move_to_aruco(self, cx, width, distance):
        twist = Twist()
        center_x = width / 2
        error_x = cx - center_x
        
        #asservissement proportionnel de la  vitesse en fonction de la distance  
        Kp_angular = 0.002  # Gain pour l'orientation
        Kp_linear = 0.5  # Gain pour l'avance

        twist.angular.z = -Kp_angular * error_x

        if distance > 0.5:  # Se déplace seulement si loin
            twist.linear.x = min(Kp_linear * distance, 0.3)  # Vitesse max 0.3 m/s
        else:
            twist.linear.x = 0.0  # Stop quand proche

        
        self.publisher.publish(twist)
        self.get_logger().info(f"Distance: {distance:.2f}, Vitesse: {twist.linear.x:.2f}, Rotation: {twist.angular.z:.2f}")        
    
        # Vérifier si on est suffisamment proche et arrêter
       

    # def stop_robot(self):
    #     stop_msg = Twist()
    #     self.publisher.publish(stop_msg)
    #     self.get_logger().info("Robot arrêté.")

def main(args=None):
    rclpy.init(args=args)
    node = TiagoPickPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
