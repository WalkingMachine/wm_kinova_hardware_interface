//
// Created by philippe on 08/05/17.
//
// Description: Petit scripte pour tester les hardwares interface des moteurs kinova du bras de Sara.
//

#include "../include/sara_arm/kinova_hardware_interface.h"



int main(int argc, char **argv) {
    // ROS
    // Initialisation
    ROS_INFO("LOAD! LOAD! LOAD!");


    //kinova_hardware_interface KHI1( "torso_arm_joint", 0);
    kinova_hardware_interface KHI2( "arm_shoulder_joint", 1);
    kinova_hardware_interface KHI3( "arm_rot_elbow_joint", 2);
    kinova_hardware_interface KHI4( "arm_elbow_joint", 3);
    //kinova_hardware_interface KHI5( "arm_rot_wrist_joint", 4);


    //KHI1.ActivateTemperatureMonitoring( argc, argv );
    KHI2.ActivateTemperatureMonitoring( argc, argv );
    KHI3.ActivateTemperatureMonitoring( argc, argv );
    KHI4.ActivateTemperatureMonitoring( argc, argv );
    //KHI5.ActivateTemperatureMonitoring( argc, argv );


    ROS_INFO("run. run! RUN!");
    while(ros::ok()){

        KHI2.Read();
        KHI3.Read();
        KHI4.Read();

        if ( (ros::Time::now().sec/2)%2 == 1 ){
            KHI2.cmd = 10;
        } else {
            KHI2.cmd = -10;
        }

        if ( (ros::Time::now().sec/3)%2 == 1 ){
            KHI3.cmd = 10;
        } else {
            KHI3.cmd = -10;
        }

        if ( (ros::Time::now().sec/4)%2 == 1 ){
            KHI4.cmd = 10;
        } else {
            KHI4.cmd = -10;
        }


        //ROS_INFO("%lf", KHI4.cmd );
        KHI2.Write();
        KHI3.Write();
        KHI4.Write();
        usleep(2000);

    }

    return 0;
}