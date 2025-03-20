import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

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

        self.searching = True  
        self.target_distance = 1.0  
        self.last_detected_id = None  

        # Variables pour déplacement jusqu'à détection de code Aruco 
        self.is_rotating = False
        self.rotation_angle = 0.0
        self.rotation_increment = 0.1  # radians

        # Publisher pour envoyer des commandes au bras
        self.arm_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
    
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
            distance = 500 / (cy + 1)

            if distance > self.target_distance + 0.2:  
                self.move_to_aruco(cx, frame.shape[1], distance)
            else:
                if self.detection_time is None:
                    self.detection_time = time.time()

                elapsed_time = time.time() - self.detection_time
                if elapsed_time >= 2:  
                    self.get_logger().info("Attente terminée, reprise de la recherche.")
                    self.detection_time = None  
                    self.last_detected_id = ids.flatten()[-1]  
                    self.wave_arm()  
                    self.move_forward_slightly()
                    self.searching = True  

        else:
            self.detection_time = None  
            self.get_logger().info("Aucun ArUco détecté, rotation du robot.")
            if self.searching:
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

        if distance > self.target_distance + 0.2:  # Ajout d'une marge de tolérance
            twist.linear.x = min(Kp_linear * (distance - self.target_distance), 0.3)  # Vitesse max 0.3 m/s
        else:
            twist.linear.x = 0.0  # Stop quand proche
            self.get_logger().info("Arrivé à 1m du marqueur, recherche d'un autre Aruco.")
            self.searching = True  # Redémarrer la recherche

        
        self.publisher.publish(twist)
        self.get_logger().info(f"Distance: {distance:.2f}, Vitesse: {twist.linear.x:.2f}, Rotation: {twist.angular.z:.2f}")        
    
        # Vérifier si on est suffisamment proche et arrêter
       

    def stop_robot(self):
        stop_msg = Twist()
        self.publisher.publish(stop_msg)
        self.get_logger().info("Robot arrêté.")

    def wave_arm(self):
        self.get_logger().info(f"Exécution du mouvement de bras pour ArUco {self.last_detected_id}")
        # Création d'un message JointTrajectory pour le mouvement de "wave"
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']  # noms des articulations

        # Ajout de points de trajectoire pour le mouvement de "wave"
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0]  # Position initiale
        point1.time_from_start = Duration(sec=0, nanosec=0)
        trajectory_msg.points.append(point1)

        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, 0.5, 0.0]  # Position intermédiaire
        point2.time_from_start = Duration(sec=1, nanosec=0)
        trajectory_msg.points.append(point2)

        point3 = JointTrajectoryPoint()
        point3.positions = [0.0, 0.0, 0.0]  # Retour à la position initiale
        point3.time_from_start = Duration(sec=2, nanosec=0)
        trajectory_msg.points.append(point3)

        # Publication du message
        self.arm_publisher.publish(trajectory_msg)
        self.get_logger().info("Fait un mouvement de wave avec le bras")

def main(args=None):
    rclpy.init(args=args)
    node = TiagoPickPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
