// Copyright[2017] <Walking Machine> [copyright]


#include "Lib/sara_arm_hardware_interface.h"
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <dlfcn.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"






MyRobot::MyRobot() {
    // connect and register the joint state interface
    int i = 0;
    std::string Name;
    ROS_INFO("\"< < <      E N R E G I S T R E M E N T   D E S   J O I N T S     > > >\"");
    Name = "torso_arm_joint";
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle( Name, &pos[i], &vel[i], &eff[i++]));
    joint_velocity_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(Name), &cmd[i++]));
    Name = "arm_shoulder_joint";
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle( Name, &pos[i], &vel[i], &eff[i++]));
    joint_velocity_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(Name), &cmd[i++]));
    Name = "arm_rot_elbow_joint";
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle( Name, &pos[i], &vel[i], &eff[i++]));
    joint_velocity_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(Name), &cmd[i++]));
    Name = "arm_elbow_joint";
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle( Name, &pos[i], &vel[i], &eff[i++]));
    joint_velocity_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(Name), &cmd[i++]));
    Name = "arm_rot_wrist_joint";
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle( Name, &pos[i], &vel[i], &eff[i++]));
    joint_velocity_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(Name), &cmd[i++]));
    Name = "arm_high_low_wrist_joint";
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle( Name, &pos[i], &vel[i], &eff[i++]));
    joint_velocity_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(Name), &cmd[i++]));
    Name = "arm_low_wrist_grasp_socket_joint";
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle( Name, &pos[i], &vel[i], &eff[i++]));
    joint_velocity_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(Name), &cmd[i++]));

    registerInterface(&joint_state_interface_);
    registerInterface(&joint_velocity_interface_);

}



void MyRobot::Read() {



    float PositionList[NOMBRE_DE_MOTEURS_KINOVA];
    float VelocityList[NOMBRE_DE_MOTEURS_KINOVA];
    MyGetActuatorsPosition(PositionList);
    MyGetActuatorsPosition(VelocityList);

    for (int i=0; i < NOMBRE_DE_MOTEURS_KINOVA; i++) {
        // << ----  U P D A T E   S T A T U S  ---- >>
        pos[i] = PositionList[i];
        vel[i] = VelocityList[i];
        eff[i] = 0.0F;
    }

}



void MyRobot::Write() {
    //ROS_INFO("%d",cmd[0]);
    TrajectoryPoint pointToSend;
    /*
        ROS_INFO("cool! %0.0f, %0.0f, %0.0f, %0.0f, %0.0f, %0.0f, %0.0f",
                 cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]);
    */
    //  << ---- E X E C U T E   O R D E R S ---- >>

    pointToSend.Position.Actuators.Actuator1 = cmd[0];
    pointToSend.Position.Actuators.Actuator2 = cmd[1];
    pointToSend.Position.Actuators.Actuator3 = cmd[2];
    pointToSend.Position.Actuators.Actuator4 = cmd[3];
    pointToSend.Position.Actuators.Actuator5 = cmd[4];
    pointToSend.SynchroType = 0;
    pointToSend.LimitationsActive = 0;
    pointToSend.Limitations.speedParameter1 = 100;
    pointToSend.Limitations.speedParameter2 = 100;
    pointToSend.Limitations.speedParameter3 = 100;
    pointToSend.Position.Type = ANGULAR_VELOCITY;
    MySendAdvanceTrajectory(pointToSend);

}





int main(int argc, char **argv) {

    bool Succes = false;
    while (!Succes) {
        // We load the library

        ROS_INFO("\"* * *            C H A R G E M E N T   D E   L A   B D           * * *\"");
        commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so", RTLD_NOW|RTLD_GLOBAL);

        // We load the functions from the library
        MyInitAPI = (int (*)()) dlsym(commandLayer_handle, "InitAPI");
        MyCloseAPI = (int (*)()) dlsym(commandLayer_handle, "CloseAPI");
        MyMoveHome = (int (*)()) dlsym(commandLayer_handle, "MoveHome");
        MyEraseAllTrajectories = (int (*)()) dlsym(commandLayer_handle, "EraseAllTrajectories");
        MySetJointZero = (int (*)(int ActuatorAdress)) dlsym(commandLayer_handle, "SetJointZero");
        MyInitFingers = (int (*)()) dlsym(commandLayer_handle, "InitFingers");
        MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
        MySetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle, "SetActiveDevice");
        MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle, "SendBasicTrajectory");
        MySendAdvanceTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle, "SendAdvanceTrajectory");
        MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
        MySetActuatorMaxVelocity = (int (*)(float &)) dlsym(commandLayer_handle, "SetActuatorMaxVelocity");
        MyGetSensorsInfo = (int (*)(SensorsInfo &)) dlsym(commandLayer_handle, "GetSensorsInfo");

        MyGetActuatorsPosition = (int (*)(float *)) dlsym(commandLayer_handle, "GetActuatorsPosition");
        MyGetAngularVelocity = (int (*)(float *)) dlsym(commandLayer_handle, "GetAngularVelocity");

        // << ----   I N I T I A L I S A T I O N   ---- >>
        if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
            (MySendAdvanceTrajectory == NULL) || (MyMoveHome == NULL) || (MyInitFingers == NULL)) {

            ROS_INFO("\"* * *        P R O B L E M E   D E   C H A R G E M E N  T        * * *\"");
            ROS_INFO("\"* * *             N O U V E L L E   T E N T A T I V E            * * *\"");
            sleep(1);
        } else {
            Succes = true;
        }
    }


    Succes = false;
    int nb_attempts = 1;
    while (!Succes) {

        ROS_INFO("\"* * *              R E C H E R C H E   D U   B R A S             * * *\"");
        result = (*MyInitAPI)();
        devicesCount = MyGetDevices(devices, result);
        if (result != 1) {

            ROS_INFO("\"* * *      I N I T I A L I S A T I O N   T E R M I N E E         * * *\"");
            ROS_INFO("\"* * *           B R A S   I N T R O U V V A B L E                * * *\"");
            ROS_INFO("\"* * *           N O U V E L L E   R E C H E R C H E              * * *\"");
            nb_attempts++;
            sleep(1);
        } else {
            Succes = true;
            ROS_INFO("\"* * *      I N I T I A L I S A T I O N   T E R M I N E E         * * *\"");
            ROS_INFO("\"* * *                  B R A S   T R O U V E E                   * * *\"");
        }
        Succes = true;

    }

    // ROS
    // Initialisation
    ros::init(argc, argv, "sara_arm_controller");

    ROS_INFO("\"* * *      spinner         * * *\"");
    ros::NodeHandle n;
    //ros::MultiThreadedSpinner spinner(2);  // Use 2 threads
    //spinner.spin();
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration period(0.02);

    // Création de l'instance de Sara
    MyRobot Sara;

    controller_manager::ControllerManager cm(&Sara, n);


    ROS_INFO("\"* * *      I N I T I A L I S A T I O N   T E R M I N E E         * * *\"");
    while (ros::ok())
    {
        Sara.Read();
        cm.update(ros::Time::now(), period);
        Sara.Write();
    }

}
