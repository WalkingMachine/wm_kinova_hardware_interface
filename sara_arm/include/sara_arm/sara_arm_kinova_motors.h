//
// Created by philippe on 16/05/17.
//

#ifndef PROJECT_SARA_ARM_KINOVA_MOTORS_H
#define PROJECT_SARA_ARM_KINOVA_MOTORS_H

#include "kinova_hardware_interface.h"

class torso_arm_joint : public kinova_hardware_interface {
public: torso_arm_joint();};

class arm_shoulder_joint : public kinova_hardware_interface {
public: arm_shoulder_joint();};

class arm_rot_elbow_joint : public kinova_hardware_interface {
public: arm_rot_elbow_joint();};

class arm_elbow_joint : public kinova_hardware_interface {
public: arm_elbow_joint();};

class arm_rot_wrist_joint : public kinova_hardware_interface {
public: arm_rot_wrist_joint();};

#endif //PROJECT_SARA_ARM_KINOVA_MOTORS_H
