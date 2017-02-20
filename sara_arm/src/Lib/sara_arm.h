#ifndef Animation_H_
#define Animation_H_

	#include <iostream>
	#include <dlfcn.h>
	#include <vector>
	#include <Kinova.API.CommLayerUbuntu.h>
	#include <KinovaTypes.h>
	#include <stdio.h>
	#include <unistd.h>
	#include <stdlib.h>
	#include <cmath>
	#include <string>
	#include "ros/ros.h"
	#include "std_msgs/String.h"
	#include "std_msgs/Int8MultiArray.h"
	#include "std_msgs/Int8.h"
	#include "std_msgs/Bool.h"
	#include <fstream>


	// Définition des structures
	struct PointDeSquence {
		int Speed;
		int Joints[10];
	};
	struct Sequence {
		PointDeSquence Points[1000];
		int Lenght;
	};

	//Handle for the library's command layer.
	void * commandLayer_handle;

	// Function pointers to the functions we need
	int (*MyInitAPI)();
	int (*MyCloseAPI)();
	int (*MySendBasicTrajectory)(TrajectoryPoint command);
	int (*MySendAdvanceTrajectory)(TrajectoryPoint command);
	int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
	int (*MySetActiveDevice)(KinovaDevice device);
	int (*MyMoveHome)();
	int (*MySetJointZero)(int ActuatorAdress);
	int (*MyInitFingers)();
	int (*MyGetAngularCommand)(AngularPosition &);
	int (*MyEraseAllTrajectories)();
	int (*MyGetSensorsInfo)(SensorsInfo &);
	int (*MySetActuatorMaxVelocity)(float &);

	// Fichier de sauvegarde d'animations
    //	std::ifstream infile("Sauvegarde.txt");

	// Nodes:
	void teleop(const std_msgs::Int8MultiArray& msg);  // Node qui lis un topic pour controler le bras en mode vélocité
	void animation( const std_msgs::String& msg );  // Node que controle les actions d'animation
	void Execute_sequence( Sequence Anim );  // Routine qui exécute une séquence pré enseigné

	// Fonctions:
	void Stop( );  // Commande pour arrêter les mouvements du bras et éffacer son FIFO
	int main(int argc, char **argv);  // Routine principale
	void MyGoToStart( );  // Positionne le bras à sa position de départ
	void WaitForReach(  );  // Attendre que le bras ateinge le prochain point du FIFO
	void ApplyPoint( int Speed );  // Ajoute un point de position au FIFO du bras
	void SetJointRelPoint( int Joint, int Angle);  // Prépare la valeur de point d'un joint de facon relative à la position actuelle.
	void SetJointGlobPoint( int Joint, int Angle );  // Prépare la valeur de point d'un joint de facon globale.
	void ApplyVelocities( );  // Ajoute un point de vélocité au FIFO du bras

#endif
