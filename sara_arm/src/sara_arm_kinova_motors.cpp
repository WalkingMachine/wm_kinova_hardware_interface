//
// Created by philippe on 16/05/17.
//

#include "../include/sara_arm/sara_arm_kinova_motors.h"

torso_arm_joint::torso_arm_joint() : kinova_hardware_interface( "torso_arm_joint", 0 ){}
arm_shoulder_joint::arm_shoulder_joint() : kinova_hardware_interface( "arm_shoulder_joint", 1 ){}
arm_rot_elbow_joint::arm_rot_elbow_joint() : kinova_hardware_interface( "arm_rot_elbow_joint", 2 ){}
arm_elbow_joint::arm_elbow_joint() : kinova_hardware_interface( "arm_elbow_joint", 3 ){}
arm_rot_wrist_joint::arm_rot_wrist_joint() : kinova_hardware_interface( "arm_rot_wrist_joint", 4 ){}