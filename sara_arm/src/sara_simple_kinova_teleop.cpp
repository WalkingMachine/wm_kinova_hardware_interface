//
// Created by philippe on 08/05/17.
//
// Description: Petit scripte controler les moteurs kinova de sara en teleop.
//

#include "../include/sara_arm/sara_arm_kinova_motors.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class ArmTeleop{
public :
    ArmTeleop( );

private:
    ros::NodeHandle nh;
    ros::Subscriber JoySub;
    kinova_hardware_interface* KHI[5];
    void JoyCB(const sensor_msgs::Joy::ConstPtr& joy );
};


ArmTeleop::ArmTeleop( ){

    KHI[0] = new torso_arm_joint();
    KHI[1] = new arm_shoulder_joint();
    KHI[2] = new arm_rot_elbow_joint();
    KHI[3] = new arm_elbow_joint();
    KHI[4] = new arm_rot_wrist_joint();

    for ( int i=0; i< 5; i++ ){
        KHI[i]->init( nh );
    }

    nh = ros::NodeHandle();

    // sub to joy
    JoySub = nh.subscribe( "joy2", 1 , &ArmTeleop::JoyCB, this );

}


void ArmTeleop::JoyCB(const sensor_msgs::Joy::ConstPtr& joy ){


    for ( int i=0; i< 5; i++ ){
        KHI[i]->Read();
    }

    KHI[0]->cmd = joy->axes[0]*-15;
    KHI[1]->cmd = joy->axes[1]*-30;
    KHI[2]->cmd = joy->axes[3]* 60;
    KHI[3]->cmd = joy->axes[4]*-60;

    for ( int i=0; i< 5; i++ ){
        KHI[i]->Write();
    }
}


int main(int argc, char **argv) {
    // ROS
    // Initialisation
    ROS_INFO("LOAD! LOAD! LOAD!");

    ros::init(argc, argv, "kinova_controller_teleop");

    ArmTeleop ArmTeleop;

    // [optionnal] init kinova status monitoring
    kinova_hardware_interface::StartStatusMonitoring( argc, argv );


    ros::spin();


    return 0;
}
