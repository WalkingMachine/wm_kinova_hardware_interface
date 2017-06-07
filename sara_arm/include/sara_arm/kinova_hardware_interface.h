//
// Created by philippe on 03/05/17.
//

#ifndef PROJECT_WMKinovaHardwareInteface_H
#define PROJECT_WMKinovaHardwareInteface_H

#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <string>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

class WMKinovaHardwareInteface : public hardware_interface::RobotHW{
public:
    // << ---- H I G H   L E V E L   I N T E R F A C E ---- >>
    // Functions
    WMKinovaHardwareInteface();
    bool init( ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh );
    static bool StartStatusMonitoring( int argc, char **argv );
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);

    // Variables
    std::string Name;
    double cmd;
    double pos;
    double vel;
    double eff;
    static double Cmd[6];
    static double Pos[6];
    static double Vel[6];
    static double Eff[6];
    static double Temperature[6];
    static double Offset[6];

private:
    // << ---- M E D I U M   L E V E L   I N T E R F A C E ---- >>
    // Functions
    static double GetPos( uint Index );
    static bool SetVel( uint Index, double Vel );
    static bool InitKinova();

    // Variables
    uint Index;
    ros::NodeHandle nh;
    ros::Publisher TemperaturePublisher;
    static ros::Publisher StatusPublisher;
    static bool StatusMonitorOn;
    static double Current;
    static double Voltage;
    static bool FreeIndex[6];
    static bool Simulation;

    // << ---- L O W   L E V E L   I N T E R F A C E ---- >>
    // Functions
    static bool SendPoint();
    static bool GatherInfo();

    // Variables
    static hardware_interface::VelocityJointInterface joint_velocity_interface_;
    static hardware_interface::JointStateInterface    joint_state_interface_;
    static bool KinovaReady;
    static bool KinovaLoaded;
    static double LastSentTime;
    static double LastGatherTime;
    static TrajectoryPoint pointToSend;
    static void * commandLayer_handle;  //Handle for the library's command layer.
    static KinovaDevice devices[MAX_KINOVA_DEVICE];

    // << ---- K I N O V A   D L ---- >>
    static int (*MyInitAPI)();
    static int (*MyCloseAPI)();
    static int (*MySendAdvanceTrajectory)(TrajectoryPoint command);
    static int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
    static int (*MyMoveHome)();
    static int (*MyInitFingers)();
    static int (*MyGetAngularCommand)(AngularPosition &);
    static int (*MyEraseAllTrajectories)();
    static int (*MyGetSensorsInfo)(SensorsInfo &);
    //static int (*MySetActuatorMaxVelocity)(float &);
    //static int (*MyGetActuatorsPosition)(float &);
    //static int (*MyGetAngularVelocity)(float &);
    //static int (*MyGetAngularTorqueCommand)(float[]  );
    static int (*MyGetAngularForce)(AngularPosition &Response);

};

#endif //PROJECT_WMKinovaHardwareInteface_H
