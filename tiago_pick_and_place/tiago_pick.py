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

# Noeud TiagoPickPlace:

class TiagoPickPlace(Node):
    def __init__(self):
        super().__init__('tiago_pick_and_place')  # init du noeud
        self.get_logger().info("Tiago Publisher node started")

        # publication sur le topic /key_vel
        self.publisher = self.create_publisher(Twist, '/key_vel', 10) 
        self.get_logger().info("Tiago Publisher key_vel started")


        # souscription au topic /head_front_camera/image
        self.subscription = self.create_subscription(Image, '/head_front_camera/image', self.image_callback, 10) 
        # print de démarrage du subscriber caméra:
        self.get_logger().info("Tiago Subscriber camera node started")

        # Publisher pour envoyer des commandes au bras
        # publication au topic /arm_controller/joint_trajectory
        self.arm_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.get_logger().info("Tiago arm wave Node Started")

        # détection du code Aruco 
        # initialisation des variables permettant la détection des codes aruco d'une taille 4x4
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Variable pour le calcul de la distancce de Tiago avec les codes Aruco
        self.searching = True  
        self.target_distance = 1.0  
        self.safe_distance = 1.2  
        self.last_detected_id = None  
        self.detection_time = None  

        self.get_logger().info("Tiago ArUco Search Node Started")

    def image_callback(self, msg):
        self.get_logger().info("Image reçue")  # Vérifier que l’image s'affiche

        #conversion des couleurs de l'image en niveau de gris
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.get_logger().info(f"Image shape: {frame.shape}") #vérifie que la frame est bien chargée

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            detected_id = ids.flatten()[-1]
            
            if detected_id == self.last_detected_id:
                self.get_logger().info(f"Déjà vu ID {detected_id}, poursuite de la recherche.")
                self.searching = True
                self.rotate_robot()
                return
            
            # Détection du dessin du marqueur sur l’image
            self.get_logger().info(f"Code ArUco détecté: {detected_id}")
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Calcul de la position du centre du marqueur
            cx, cy = np.mean(corners[0][0], axis=0)
            #distance approchée : 
            distance = 500 / (cy + 1)

            # condition de distance entre Tiago et le code Aruco 
            # si il est trop près alors il fait une translation vers l'arrière
            if distance > self.safe_distance:  
                self.move_to_aruco(cx, frame.shape[1], distance)
            elif distance < self.target_distance:  
                self.move_backward_slightly()  
            else:
                if self.detection_time is None:
                    self.detection_time = time.time()

                elapsed_time = time.time() - self.detection_time
                if elapsed_time >= 2:
                    self.get_logger().info("ID traité, passage à la recherche suivante.")
                    self.detection_time = None
                    self.last_detected_id = detected_id  
                    self.wave_arm()  
                    self.move_laterally()  
                    self.searching = True  

        else:
            self.detection_time = None  
            self.get_logger().info("Aucun ArUco détecté, rotation du robot.")
            if self.searching:
                self.rotate_robot()

        cv2.imshow("Tiago Camera - Aruco Detection", frame)
        cv2.waitKey(1)

    #rotation sur lui même quand pas de détéection de code Aruco 
    def rotate_robot(self):
        twist = Twist()
        twist.angular.z = 0.3  
        self.publisher.publish(twist)
        self.get_logger().info("Rotation en cours...")

    # fonction de déplacement vers les tags Aruco en fonction d'un calcul de PID en fonction de la vitesse et de la distance
    def move_to_aruco(self, cx, width, distance):
        twist = Twist()
        center_x = width / 2
        error_x = cx - center_x

        #définition des coeficients 
        Kp_angular = 0.002
        Kp_linear = 0.5  

        #asservissement proportionnel de la  vitesse en fonction de la distance  
        twist.angular.z = -Kp_angular * error_x    # Gain pour l'orientation
        twist.linear.x = min(Kp_linear * (distance - self.target_distance), 0.2)  # Gain pour avancer

        self.publisher.publish(twist)
        self.get_logger().info(f"Approche ArUco: Distance={distance:.2f}, Vitesse={twist.linear.x:.2f}")

    # pour maintenir une distance à 1m des tags Aruco il fait des petits mouvement vers l'arrière pour garder ses distances 
    def move_backward_slightly(self):
        twist = Twist()
        twist.linear.x = -0.1  
        self.publisher.publish(twist)
        self.get_logger().info("Recule légèrement pour maintenir 1m.")

    # fait un léger déplacement latéral après une détection.
    def move_laterally(self):
        twist = Twist()
        twist.linear.y = 0.2  # Déplacement latéral pour sortir de l'axe du marqueur
        self.publisher.publish(twist)
        time.sleep(1)
        self.stop_robot()
        self.get_logger().info("Déplacement latéral effectué.")

    # fait un mouvement de "coucou" avec son bras lorsqu'il a vu son dernier ID (mais ne fonctionne pas, cette fonction n'est donc pas active)
    def wave_arm(self):
        self.get_logger().info(f"Exécution du mouvement de bras pour ArUco {self.last_detected_id}")

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']  

        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0]  
        point1.time_from_start = Duration(sec=0, nanosec=0)
        trajectory_msg.points.append(point1)

        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, -0.5, 0.0]  
        point2.time_from_start = Duration(sec=1, nanosec=0)
        trajectory_msg.points.append(point2)

        point3 = JointTrajectoryPoint()
        point3.positions = [0.0, 0.0, 0.0]  
        point3.time_from_start = Duration(sec=2, nanosec=0)
        trajectory_msg.points.append(point3)

        self.arm_publisher.publish(trajectory_msg)
        self.get_logger().info("Mouvement du bras exécuté.")

    # arrêt du robot en publiant sur les commandes des moteurs 
    def stop_robot(self):
        stop_msg = Twist()
        self.publisher.publish(stop_msg)
        self.get_logger().info("Robot arrêté.")

def main(args=None):
    rclpy.init(args=args)
    node = TiagoPickPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
