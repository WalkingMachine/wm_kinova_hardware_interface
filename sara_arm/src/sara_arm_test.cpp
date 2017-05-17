//
// Created by philippe on 08/05/17.
//
// Description: Petit scripte pour tester les hardwares interface des moteurs kinova du bras de Sara.
//

#include "../include/sara_arm/sara_arm_kinova_motors.h"

int main(int argc, char **argv) {
    // ROS
    // Initialisation
    ROS_INFO("LOAD! LOAD! LOAD!");

    ros::init(argc, argv, "test_kinova_controller");
    ros::NodeHandle nh;

    auto KHI1 = torso_arm_joint();
    //auto KHI2 = arm_shoulder_joint();
    //auto KHI3 = arm_rot_elbow_joint();
    //auto KHI4 = arm_elbow_joint();
    //auto KHI5 = arm_rot_wrist_joint();



    KHI1.init( nh );
    //KHI2.init( nh );
    //KHI3.init( nh );
    //KHI4.init( nh );
    //KHI5.init( nh );


    kinova_hardware_interface::StartStatusMonitoring( argc, argv );

    ros::spinOnce();


    ROS_INFO("run. run! RUN!");
    while(ros::ok()){

        KHI1.Read();

        if ( (ros::Time::now().sec/2)%2 == 1 ){
            KHI1.cmd = 10;
        } else {
            KHI1.cmd = -10;
        }

        KHI1.Write();

        usleep(2000);

    }

    return 0;
}
