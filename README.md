# iFollow_test

Question 1 : Mise en place de l'environnement de test
  
  
Question 2 : Multiplexeur
  Comme je n'ai pas réussi à mettre tout le folder du turtlebot3 sur le git, je pense que mon implémentation de cette question n'est pas visible.
  Cependant j'ai bien implémenté un mutiplexeur avec la commande suivante : rosrun topic_tools mux cmd_velo cmd_local cmd_web mux:=mux_cmdvel
  Les informations concernant ce node sont visible dans un screenshot que j'ai fourni.
  Pour switcher de source de commande, il faut utiliser "mux_select MUX_NAME <topic>", ce qui dans mon cas était "mux_select /cmd_velo_mux_1670499400017044505 topic:"cmd_web". Evidement cela renvoie une erreur parce que les topics cmd_local et cmd_web n'existent pas ici.
