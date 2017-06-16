# wm_kinova_hardware_interface

Package pour le hardware interface des moteurs kinova de sara.
Ce package agit comme un plugin pour sara_control. Par conséquent, aucun launch direct n'est néssésaire.
Pour tester ce package:
  - cloner le repo sara_launch dans votre workspace
  - cloner le repo sara_control dans votre workspace
  - cloner le repo sara_description dans votre workspace
  - cloner le repo robotiq_c2_description dans votre workspace
  - $apt instal ros-kinetic-ros-control
  - $catkin_make
  - $roslaunch sara_description sara_description
  - $roslaunch sara_control sara_control
  - $roslaunch sara_launch sara_control

les controleur sara_arm_trajectory_controller et sara_arm_velocity_controller devrais alors être disponnible pour ere loader avec la commande $rosservice call /controller_manager/load_controller "name: '<le_nom_du_controlleur>'"
  
