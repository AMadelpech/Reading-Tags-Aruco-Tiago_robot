# Détection et suivi d'un tag ArUco avec le Robot Tiago

**Semestre 2: Cours de Robotique de service**

<img src="/home/rosdev/ros2_ws/src/tiago_pick_and_place/image/image.png"> 


👨‍💻 **Etudiants:** Célia Gonzalez & Aimy Madelpech

👨‍🎓 **Promotion:** Master 2 Robotique et ingénieurie des systèmes embarqués spécialité mécatronique 

## 📄 Résumé du projet
Le but du projet a été de créer un package sous ROS2 en python afin de programmer un robot Tiago de détectection de tag (ou markers) ARUCo à l'aide de sa caméra.
Puis, de permettre au robot de s'orienter et de se déplacer vers un tag ARUCo.

Le nom du paquet et du noeud sont nommé pick and place car l'idée initiale du projet était de faire une chaîne de pick&place par soucis de temps nous nous sommes concentré sur la lecture des tags Aruco. 

## 📌 Les objectifs du projets:
• Utiliser OpenCV et ROS2 pour traiter les images 

• Lecture et conversion d’images avec cv2

• Orientation du robot dans le simulateur

• Déplacer le robot en direction 2d et en orientation grâce au topic /key_vel publié sur le noeud TiagoPickPlace

• Détecter les tags ArUco en temps réel via la souscription du topic /head_front_camera/image vers le noeud

• Ajuster la position du robot en fonction de la détection du tag avec des calculs de PID de la distance en fonction de la vitesse dans la fonction move_to_aruco

• Faire un mouvement de bras devant le tag (cette fonctionalité a été programmé mais pas utiliser donc tocpic inactifs)

## 🚀 Les pré-requis
### Instructions d'installation

Notre projet nécessite un robot Tiago fonctionnel simulé dans Tiago Harmonic, installé à partir de: https://github.com/Tiago-Harmonic/tiago_harmonic

Le projet et ses dépendences peuvent être installé via ces lignes de commande:
```bash
git clone https://github.com/AMadelpech/Reading-Tags-Aruco-Tiago_robot.git
```

### Instruction de démarrage

Tout d'abord dans un terminal, se placer dans son workspace respectif commme `ros2_ws` par exemple: 
```bash
cd ros2_ws
```

Ensuite démarrage de l'environnement de simulation de Tiago dans le monde `pick_and_place` :
```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=pick_and_place
```

Enfin, lancement de notre noeud  **TiagoPickPlace**:
```bash
ros2 run tiago_pick_and_place tiago_pick 
```


## 📚 Références and bibliographie
• Documentation ROS 2

• Tutoriels Tiago Robot

• Ressources GitLab et GitHub liées à la version par ordinateur et à la navigation autonome

• Ressource Gitlab : Approcher un tag ARUco vu sur un mur à distance https://gitlab.com/-/snippets/4826671 
