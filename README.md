# iFollow_test

J'ai eu des problèmes pour télécharger l'ensemble du fichier turtlebot3 sur Github. Je n'ai donc fourni que les scripts python que j'ai écrit.
Le script mqtt/mqtt_publisher.py était placé dans un dossier à part, indépendant de ros (le dossier mqtt).
Les scripts mqtt_subscriber.py et apriltags.py était placé dans le package ROS du turtlebot3, dans le folder src/turtlebot3/turtlebot3_teleop/nodes.
Pour refaire les simulations et vérifier que mon code fonctionne bien, il faudrait sûrement les replacer à cet endroit.



Question 1 : Mise en place de l'environnement de test (temps estimé : 1 heure)
	On peut lancer les simulations après avoir installé ROS, Gazebo et le package turtlebot3. Il faut aussi préciser qu'on utilise un turtlebot3 de type burger (export TURTLEBOT3_MODEL=burger) quand on ouvre un terminal.  On lance ensuite la simulation gazebo (roslaunch turtlebot3_gazebo turtlebot3_world.launch). On peut ensuite téléopérer le robot au clavier en lançant roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch (on contrôle le robot avec les touches wqsdz). En lançant le fichier slam (roslaunch turtlebot3_slam turtlebot3_slam.launch slam_method:=gmapping) avant la téléopération, on lance rviz qui permet de voir la vision du robot : cela permet, en déplaçant le robot grâce à la téléopération, de faire un mapping de l'espace de travail du robot. On obtient la carte que j'ai fourni (map.pgm). Enfin, en utilisant la carte obtenue pour lancer le fichier roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml, on peut donner un goal au robot : grâce à l'interface de Rviz, on place le robot (2D Pose Estimate, position (x,y) + orientation), puis on lui donne un goal (2D Nav Goal), et le robot le rejoint.
	
  
Question 2 : Multiplexeur (temps estimé : 1 heure)
	Comme je n'ai pas réussi à mettre tout le folder du turtlebot3 sur le git, je pense que mon implémentation de cette question n'est pas visible.
  	Cependant j'ai bien implémenté un mutiplexeur avec la commande suivante : rosrun topic_tools mux cmd_velo cmd_local cmd_web mux:=mux_cmdvel
  	Les informations concernant ce node sont visible dans un screenshot que j'ai fourni.
  	Pour switcher de source de commande, il faut utiliser "mux_select MUX_NAME <topic>", ce qui dans mon cas était "mux_select 			/cmd_velo_mux_1670499400017044505 topic:"cmd_web". Evidement cela renvoie une erreur parce que les topics cmd_local et cmd_web n'existent pas ici.
  
  
Question 3 : Téléopération à distance (temps estimé : environ 3h, surtout le temps de comprendre ce qu'est le protocole mqtt et comment l'implémenter avec ROS)
	Les scripts mqtt/mqtt_publisher.py et src/turtlebot3/turtlebot3_teleop/nodes/mqtt_subscriber.py réalisent cette fonction. J'ai décidé de transmettre l'information sous forme de string plutôt que de float parce que cela me semblait plus simple. Sinon, rien de spécial, tout fonctionne bien.
	

Question 4-5-6 : AprilTags (temps estimé : 3h pour la question 4, beaucoup de difficultés avec les AprilTags. 1h pour les questions 5+6, c'était rapide, tout marchait bien)
	Le script src/turtlebot3/turtlebot3_teleop/nodes/apriltags.py est utilisé ici. Peut être qu'il aurait fallu le placer dans un autre dossier que teleop, mais comme le but de ces questions est seulement de publier un topic avec le nav_goal sans savoir où sont ses subscriber, cela n'a pas d'importance.
	J'ai mis un parser pour choisir le nom de l'image à analyser (-i path_to_image), sinon une image par défaut est analysée (mais cela correspondait au path sur mon pc, qui est sûrement différent de celui sur votre ordinateur, donc il faut changer le path). Un autre parser --webcam peut être utilisé pour prendre une photo avec la webcam (en appuyant sur SPACE) et analyser s'il y a un apriltag dessus. 
	
