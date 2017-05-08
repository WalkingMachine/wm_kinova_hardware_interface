// Copyright[2017] <Walking Machine> [copyright]


#include "Lib/sara_arm_hardware_interface.h"
#include <controller_manager/controller_manager.h>
#include <dlfcn.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"

ArmHardware::ArmHardware() {
    // connect and register the joint state interface
    int i = 0;
    ROS_INFO("\"* * *      E N R E G I S T R E M E N T   D E S   J O I N T S     * * *\"");

    //Declare all joints to ros control
    for (i=0; i <= 6; i++) {
        ROS_INFO("\"* * *                   J O I N T   #%d                           * * *\"", i );
        MaPos[i] = MaVel[i] = cmd[i] = pos[i] = vel[i] = eff[i] = 0;

        hardware_interface::JointStateHandle handle( Names[i], &pos[i], &vel[i], &eff[i]);
        joint_state_interface_.registerHandle(handle);
        joint_velocity_interface_.registerHandle(hardware_interface::JointHandle(handle, &cmd[i]));

    }
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_velocity_interface_);

    // Set general kinova Points parameters.
    pointToSend.Limitations.accelerationParameter1 = 100;
    pointToSend.Limitations.accelerationParameter2 = 100;
    pointToSend.Limitations.accelerationParameter3 = 100;
    pointToSend.Limitations.speedParameter1 = 100;
    pointToSend.Limitations.speedParameter2 = 100;
    pointToSend.Limitations.speedParameter3 = 100;
    pointToSend.SynchroType = 0;
    pointToSend.LimitationsActive = 1;
    pointToSend.Limitations.speedParameter1 = 60;
    pointToSend.Limitations.speedParameter2 = 60;
    pointToSend.Limitations.speedParameter3 = 60;
    pointToSend.Position.Type = ANGULAR_VELOCITY;
}

void ArmHardware::Read() {
    for (int i = NOMBRE_DE_MOTEURS_KINOVA; i <= 6; i++) {
        vel[i] = MaVel[i];
        MaPos[i] = MaPos[i] + vel[i]/100.0;
        if (MaPos[i] < -180) {
            MaPos[i] += 360;
        }
        if (MaPos[i] > 180) {
            MaPos[i] -= 360;
        }
        pos[i] = MaPos[i];
        eff[i] = 0.0F;
    }


    if ( NOMBRE_DE_MOTEURS_KINOVA == 5 ) {
        AngularPosition PositionList;
        //float VelocityList[NOMBRE_DE_MOTEURS_KINOVA];
        MyGetAngularCommand(  PositionList );
        //MyGetActuatorsPosition(VelocityList);

        pos[0] = PositionList.Actuators.Actuator1/180*3.14159+Offsets[0];
        pos[1] = PositionList.Actuators.Actuator2/180*3.14159+Offsets[1];
        pos[2] = PositionList.Actuators.Actuator3/180*3.14159+Offsets[2];
        pos[3] = PositionList.Actuators.Actuator4/180*3.14159+Offsets[3];
        pos[4] = PositionList.Actuators.Actuator5/180*3.14159+Offsets[4];
        pos[5] = PositionList.Actuators.Actuator6/180*3.14159+Offsets[5];
    }

 //   ROS_INFO("pos! %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f",
 //            pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]);
}



void ArmHardware::Write() {
    for (int i=0; i <= 6; i++) {
        MaVel[i] = fmin( fmax( cmd[i], -20.0 ), 20.0 );
    }
 //   ROS_INFO("cmd! %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f",
 //            MaVel[0], MaVel[1], MaVel[2], MaVel[3], MaVel[4], MaVel[5], MaVel[6]);

    //  << ---- E X E C U T E   O R D E R S ---- >>
    if (NOMBRE_DE_MOTEURS_KINOVA == 5) {
        pointToSend.Position.Actuators.Actuator1 = (float) MaVel[0];
        pointToSend.Position.Actuators.Actuator2 = (float) MaVel[1];
        pointToSend.Position.Actuators.Actuator3 = (float) MaVel[2];
        pointToSend.Position.Actuators.Actuator4 = (float) MaVel[3];
        pointToSend.Position.Actuators.Actuator5 = (float) MaVel[4];
        MyEraseAllTrajectories();
        MySendAdvanceTrajectory(pointToSend);
    }
}


int main(int argc, char **argv) {

    bool Success = false;
    while (!Success) {
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

        MyGetActuatorsPosition = (int (*)(float &)) dlsym(commandLayer_handle, "GetActuatorsPosition");
        MyGetAngularVelocity =   (int (*)(float &)) dlsym(commandLayer_handle, "GetAngularVelocity");

        // << ----   I N I T I A L I S A T I O N   ---- >>
        if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) /*|| (MyGetActuatorsPosition == NULL) */ ||
            (MySendAdvanceTrajectory == NULL) || (MyMoveHome == NULL) || (MyInitFingers == NULL)) {

            ROS_INFO("\"* * *        P R O B L E M E   D E   C H A R G E M E N  T        * * *\"");
            ROS_INFO("\"* * *             N O U V E L L E   T E N T A T I V E            * * *\"");
            sleep(1);
        } else {
            Success = true;
        }
    }


    Success = false;
    int nb_attempts = 2;
    ROS_INFO("\"* * *              R E C H E R C H E   D U   B R A S             * * *\"");
    while (!Success) {

        result = (*MyInitAPI)();
        devicesCount = MyGetDevices(devices, result);
        if (result != 1 ) {
            if ( nb_attempts > 9 ){
                ROS_INFO("\"* * *          M O D E   S I M U L A T I O N   A C T I V E       * * *\"");
                Success = true;
            } else {
                ROS_INFO("\"* * *           B R A S   I N T R O U V V A B L E                * * *\"");
                ROS_INFO("\"* * *                 T E N T A T I V E   #%d                     * * *\"", nb_attempts );
                nb_attempts++;
                sleep(1);
            }
        } else {
            Success = true;
            ROS_INFO("\"* * *      I N I T I A L I S A T I O N   T E R M I N E E         * * *\"");
            ROS_INFO("\"* * *                  B R A S   T R O U V E E                   * * *\"");
        }
        Success = true;

    }

    // ROS
    // Initialisation
    ros::init(argc, argv, "sara_arm_controller");

    ros::NodeHandle n;

    n.getParam("/sara_arm_trajectory_controller/Offsets", Offsets);
    n.getParam("/sara_arm_trajectory_controller/Names", Names);

    //ros::MultiThreadedSpinner spinner(2);  // Use 2 threads
    //spinner.spin();
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration period(0.001);

    // Création de l'instance de SaraArm
    ArmHardware SaraArm;

    controller_manager::ControllerManager cm(&SaraArm, n);
    //ros::Time Temp = ros::Time::now();
    ROS_INFO("\"* * *      I N I T I A L I S A T I O N   T E R M I N E E         * * *\"");
    ROS_INFO("\"* * *                     D E M A R A G E                        * * *\"");
    while (ros::ok()) {
        SaraArm.Read();
        cm.update(ros::Time::now(), period);
        SaraArm.Write();
        usleep(6000);
    }

}
