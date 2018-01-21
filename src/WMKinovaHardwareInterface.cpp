//
// Created by philippe on 03/05/17.
//

#include "WMKinovaHardwareInterface.h"
#include <std_msgs/Float32.h>
#include <iostream>


namespace wm_kinova_hardware_interface {
    const uint PERIOD = 6000000;

// << ---- S T A T I C   V A R I A B L E   I N I T I A L I Z A T I O N ---- >>
    bool WMKinovaHardwareInterface::KinovaReady = false;
    bool WMKinovaHardwareInterface::KinovaLoaded = false;
    double WMKinovaHardwareInterface::LastSentTime = 0;
    double WMKinovaHardwareInterface::LastGatherTime = 0;
    double WMKinovaHardwareInterface::Current = 0;
    double WMKinovaHardwareInterface::Voltage = 0;
    bool WMKinovaHardwareInterface::FreeIndex[6];
    double WMKinovaHardwareInterface::Pos[6];
    double WMKinovaHardwareInterface::Vel[6];
    double WMKinovaHardwareInterface::Eff[6];
    double WMKinovaHardwareInterface::Cmd[6];
    double WMKinovaHardwareInterface::Offset[6];
    double WMKinovaHardwareInterface::Temperature[6];
    hardware_interface::VelocityJointInterface WMKinovaHardwareInterface::joint_velocity_interface_;
    hardware_interface::JointStateInterface    WMKinovaHardwareInterface::joint_state_interface_;
    TrajectoryPoint WMKinovaHardwareInterface::pointToSend;
    ros::Publisher WMKinovaHardwareInterface::StatusPublisher;
    void *WMKinovaHardwareInterface::commandLayer_handle;  //Handle for the library's command layer.
    KinovaDevice WMKinovaHardwareInterface::devices[MAX_KINOVA_DEVICE];

    int (*WMKinovaHardwareInterface::MyInitAPI)();
    int (*WMKinovaHardwareInterface::MyCloseAPI)();
    int (*WMKinovaHardwareInterface::MySendAdvanceTrajectory)(TrajectoryPoint command);
    int (*WMKinovaHardwareInterface::MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
    int (*WMKinovaHardwareInterface::MyMoveHome)();
    int (*WMKinovaHardwareInterface::MyGetSensorsInfo)(SensorsInfo &);
    int (*WMKinovaHardwareInterface::MyInitFingers)();
    int (*WMKinovaHardwareInterface::MyGetAngularCommand)(AngularPosition &);
    int (*WMKinovaHardwareInterface::MyGetAngularForce)(AngularPosition &Response);
    int (*WMKinovaHardwareInterface::MyEraseAllTrajectories)();
    bool WMKinovaHardwareInterface::StatusMonitorOn = false;
    bool WMKinovaHardwareInterface::Simulation = false;

// << ---- H I G H   L E V E L   I N T E R F A C E ---- >>

    bool WMKinovaHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
        if (!KinovaLoaded) {
            KinovaLoaded = true;
            KinovaLoaded = InitKinova();
            KinovaReady = true;
        }
        using namespace hardware_interface;
        Name = "";
        Index = 0;
        std::vector<std::string> Joints;
        if (!robot_hw_nh.getParam("joints", Joints)) {
            return false;
        }
        Name = Joints[0];
        if (!robot_hw_nh.getParam("index", Index)) {
            return false;
        }
        if (!robot_hw_nh.getParam("offset", Offset[Index])) {
            return false;
        }
        cmd = 0;
        pos = 0;
        vel = 0;
        eff = 0;
        FreeIndex[Index] = false;

        joint_state_interface_.registerHandle(JointStateHandle(Name, &pos, &vel, &eff));
        joint_velocity_interface_.registerHandle(JointHandle(joint_state_interface_.getHandle(Name), &cmd));
        registerInterface(&joint_state_interface_);
        registerInterface(&joint_velocity_interface_);

        TemperaturePublisher = nh.advertise<diagnostic_msgs::DiagnosticStatus>("diagnostics", 100);

        return true;
    }

    void WMKinovaHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {

        GetInfos();
//        std::cout << "\nIndex = " << Index << ", Position = " << Pos[Index] << ", Effort = " << Eff[Index];
        pos = AngleProxy( 0, Pos[Index]);
        eff = Eff[Index];
        diagnostic_msgs::DiagnosticStatus message;
        message.name = Name;
        message.hardware_id = Name;


        diagnostic_msgs::KeyValue KV1;
        KV1.key = "temperature";
        char chare[50];
        std::sprintf(chare, "%lf", Temperature[Index]);
        KV1.value = chare;

        diagnostic_msgs::KeyValue KV2;
        KV2.key = "torque";
        std::sprintf(chare, "%lf", Eff[Index]);
        KV2.value = chare;

        message.values = {KV1, KV2};
        TemperaturePublisher.publish(message);
    }

    void WMKinovaHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {
        SetVel(Index, cmd*57.295779513); // from r/s to ded/p
    }

// << ---- M E D I U M   L E V E L   I N T E R F A C E ---- >>
    bool WMKinovaHardwareInterface::GetInfos() {
        double Now = ros::Time::now().toNSec();
        bool result;  // true = no error
        if (LastGatherTime < Now - PERIOD) {
            LastGatherTime = Now;
            result = GatherInfo();
            if (!result) {
                ROS_ERROR("Kinova Hardware Interface.  error detected while trying to gather information");
                return false;
            }
        }
        return true;
    }

    bool WMKinovaHardwareInterface::SetVel(int Index, double cmd) {
        double Now = ros::Time::now().toNSec();
        bool result = true;  // true = no error
        Cmd[Index] = cmd;
        if (LastSentTime < Now - PERIOD) {
            LastSentTime = Now;
            result = SendPoint();
            if (!result) {
                ROS_ERROR("Kinova Hardware Interface.  error detected while trying to send point");
            }
        }
        return result;
    }

// << ---- L O W   L E V E L   I N T E R F A C E ---- >>
    bool WMKinovaHardwareInterface::InitKinova() {
        for (int i = 0; i < 16; i++) {
            FreeIndex[i] = true;
        }
        pointToSend.InitStruct();
        // Set general kinova Points parameters.
        pointToSend.Limitations.accelerationParameter1 = 100;
        pointToSend.Limitations.accelerationParameter2 = 100;
        pointToSend.Limitations.accelerationParameter3 = 100;
        pointToSend.Limitations.speedParameter1 = 100;
        pointToSend.Limitations.speedParameter2 = 100;
        pointToSend.Limitations.speedParameter3 = 100;
        pointToSend.SynchroType = 0;
        pointToSend.LimitationsActive = 0;
        pointToSend.Limitations.speedParameter1 = 60;
        pointToSend.Limitations.speedParameter2 = 60;
        pointToSend.Limitations.speedParameter3 = 60;
        pointToSend.Position.Type = ANGULAR_VELOCITY;

        bool Success = false;
        while (!Success) {
            // We load the kinova library

            ROS_INFO("\"* * *            C H A R G E M E N T   D E   L A   B D           * * *\"");
            commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);

            // We load the functions from the library
            MyInitAPI = (int (*)()) dlsym(commandLayer_handle, "InitAPI");
            MyCloseAPI = (int (*)()) dlsym(commandLayer_handle, "CloseAPI");
            MyMoveHome = (int (*)()) dlsym(commandLayer_handle, "MoveHome");
            MyGetSensorsInfo = (int (*)(SensorsInfo &)) dlsym(commandLayer_handle, "GetSensorsInfo");
            MyEraseAllTrajectories = (int (*)()) dlsym(commandLayer_handle, "EraseAllTrajectories");
            MyInitFingers = (int (*)()) dlsym(commandLayer_handle, "InitFingers");
            MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,
                                                                                                 "GetDevices");
            MySendAdvanceTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle, "SendAdvanceTrajectory");
            MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
            MyGetAngularForce = (int (*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularForce");

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
        int result;
        ROS_INFO("\"* * *              R E C H E R C H E   D U   B R A S             * * *\"");
        while (!Success) {

            result = (*MyInitAPI)();
            MyGetDevices(devices, result);
            if (result != 1) {
                if (nb_attempts > 4) {
                    ROS_INFO("\"* * *                   B R A S   I N T R O U V E                * * *\"");
                    ROS_INFO("\"* * *          M O D E   S I M U L A T I O N   A C T I V E       * * *\"");
                    Simulation = true;
                    Success = true;
                } else {
                    ROS_INFO("\"* * *             B R A S   I N T R O U V A B L E                * * *\"");
                    ROS_INFO("\"* * *                 T E N T A T I V E   #%d/8                   * * *\"",
                             nb_attempts);
                    nb_attempts++;
                    sleep(1);
                }
            } else {
                Success = true;
                ROS_INFO("\"* * *      I N I T I A L I S A T I O N   T E R M I N E E         * * *\"");
                ROS_INFO("\"* * *                  B R A S   T R O U V E                     * * *\"");
            }
        }
        LastGatherTime = ros::Time::now().toNSec();
        return true;  // TODO  detect errors
    }

    bool WMKinovaHardwareInterface::StartStatusMonitoring(int argc, char **argv) {
        StatusMonitorOn = true;
        std::string NodeName = "kinova status";
        ros::init(argc, argv, NodeName);
        ros::NodeHandle n;
        StatusPublisher = n.advertise<diagnostic_msgs::DiagnosticStatus>("diagnostics", 100);
        ros::spinOnce();
        return true;
    }

    bool WMKinovaHardwareInterface::GatherInfo() {

        if (KinovaReady) {
            if (Simulation) {
                // Do crude simulation
                for (int i = 0; i < 6; i++) {
                    Temperature[i] = 0.1234;
                    Pos[i] += Vel[i] / 50000;
                }
                Current = -1;
                Voltage = -1;
            } else {
                AngularPosition PositionList;
                MyGetAngularCommand(PositionList);
                Pos[0] = PositionList.Actuators.Actuator1 / 160 * M_PI - Offset[0];
                Pos[1] = PositionList.Actuators.Actuator2 / 180 * M_PI - Offset[1];
                Pos[2] = PositionList.Actuators.Actuator3 / 180 * M_PI - Offset[2];
                Pos[3] = PositionList.Actuators.Actuator4 / 180 * M_PI - Offset[3];
                Pos[4] = PositionList.Actuators.Actuator5 / 180 * M_PI - Offset[4];
                Pos[5] = PositionList.Actuators.Actuator6 / 180 * M_PI - Offset[5];

                SensorsInfo SI;
                MyGetSensorsInfo(SI);
                Temperature[0] = SI.ActuatorTemp1;
                Temperature[1] = SI.ActuatorTemp2;
                Temperature[2] = SI.ActuatorTemp3;
                Temperature[3] = SI.ActuatorTemp4;
                Temperature[4] = SI.ActuatorTemp5;
                Temperature[5] = SI.ActuatorTemp6;
                Current = SI.Current;
                Voltage = SI.Voltage;

                AngularPosition ForceList;
                MyGetAngularForce(ForceList);
                Eff[0] = ForceList.Actuators.Actuator1;
                Eff[1] = ForceList.Actuators.Actuator2;
                Eff[2] = ForceList.Actuators.Actuator3;
                Eff[3] = ForceList.Actuators.Actuator4;
                Eff[4] = ForceList.Actuators.Actuator5;
                Eff[5] = ForceList.Actuators.Actuator6;

            }
            if (StatusMonitorOn) {
                diagnostic_msgs::DiagnosticStatus message;
                message.name = "kinova_arm";
                message.hardware_id = "kinova_arm";
                diagnostic_msgs::KeyValue KV1;
                KV1.key = "current";
                char chare[50];
                std::sprintf(chare, "%lf", Current);
                KV1.value = chare;

                diagnostic_msgs::KeyValue KV2;
                KV2.key = "voltage";
                std::sprintf(chare, "%lf", Voltage);
                KV2.value = chare;

                message.values = {KV1, KV2};
                StatusPublisher.publish(message);

            }
        }
        return true;  // TODO  detect errors
    }

    bool WMKinovaHardwareInterface::SendPoint() {

        if (KinovaReady) {
            for (int i = 0; i < 6; i++) {
                Vel[i] = Cmd[i];
            }
            if (Simulation) {
                // Do crude simulation
                for (int i = 0; i < 6; i++) {
                    Temperature[i] = 0;
                }
            } else {
                //  execute order
                pointToSend.Position.Actuators.Actuator1 = (float) Cmd[0];
                pointToSend.Position.Actuators.Actuator2 = (float) Cmd[1];
                pointToSend.Position.Actuators.Actuator3 = (float) Cmd[2];
                pointToSend.Position.Actuators.Actuator4 = (float) Cmd[3];
                pointToSend.Position.Actuators.Actuator5 = (float) Cmd[4];
                pointToSend.Position.Actuators.Actuator6 = (float) Cmd[5];


                MyEraseAllTrajectories();
                //ROS_INFO( "Send!" );
                //ROS_INFO("S1=%lf", Vel[0] );
                //ROS_INFO("S2=%lf", pointToSend.Position.Actuators.Actuator1 );
                MySendAdvanceTrajectory(pointToSend);
            }
        }
        return true;  // TODO  detect errors
    }
}
PLUGINLIB_EXPORT_CLASS( wm_kinova_hardware_interface::WMKinovaHardwareInterface, hardware_interface::RobotHW)
