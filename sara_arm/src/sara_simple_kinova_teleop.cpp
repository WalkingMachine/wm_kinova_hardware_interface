//
// Created by philippe on 08/05/17.
//
// Description: Petit scripte controler les moteurs kinova de sara en teleop.
//

#include "../include/sara_arm/sara_arm_kinova_motors.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

class ArmTeleop{
public :
    ArmTeleop( );

private:
    bool EStop;
    bool TeleopOn;
    ros::NodeHandle nh;
    kinova_hardware_interface* KHI[5];
    void JoyCB(const sensor_msgs::Joy::ConstPtr& joy );
    void EStopCB(const std_msgs::Bool );
};


ArmTeleop::ArmTeleop( ){

    KHI[0] = new torso_arm_joint();
    KHI[1] = new arm_shoulder_joint();
    KHI[2] = new arm_rot_elbow_joint();
    KHI[3] = new arm_elbow_joint();
    KHI[4] = new arm_rot_wrist_joint();

    TeleopOn = false;

    for ( int i=0; i< 5; i++ ){
        KHI[i]->init( nh );
    }

    nh = ros::NodeHandle();

    // sub
    nh.subscribe( "joy2", 1 , &ArmTeleop::JoyCB, this );
    nh.subscribe( "estop", 1 , &ArmTeleop::EStopCB, this );

}


void ArmTeleop::JoyCB(const sensor_msgs::Joy::ConstPtr& joy ){

    if (TeleopOn && !EStop ) {
        for ( int i=0; i< 5; i++ ){
            KHI[i]->Read();
        }
        KHI[0]->cmd = joy->axes[0]*-15;
        KHI[1]->cmd = joy->axes[1]*-25;
        KHI[2]->cmd = joy->axes[3]* 30;
        KHI[3]->cmd = joy->axes[4]*-30;
        KHI[4]->cmd = ((joy->axes[2]) - (joy->axes[5])) * -30;

        for ( int i=0; i< 5; i++ ){
            KHI[i]->Write();
        }
    } else if ( joy->axes[2]  <= -0.999 && joy->axes[5] <= -0.999 ){
        TeleopOn = true;
        EStop = false;
    }

}

void ArmTeleop::EStopCB(const std_msgs::Bool msg ){
    ROS_INFO( "estop: %c", msg.data  );
    EStop = ( msg.data == '1' );
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