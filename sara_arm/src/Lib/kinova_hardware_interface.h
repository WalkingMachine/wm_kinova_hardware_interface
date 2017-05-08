//
// Created by philippe on 03/05/17.
//

#ifndef PROJECT_KINOVA_HARDWARE_INTERFACE_H
#define PROJECT_KINOVA_HARDWARE_INTERFACE_H

#include <Kinova.API.CommLayerUbuntu.h>
#include <KinovaTypes.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <string>
//#include <ros>


class kinova_hardware_interface{
public:
    // << ---- H I G H   L E V E L   I N T E R F A C E ---- >>
    kinova_hardware_interface(std::string Name, uint Index);
    std::string Name;
    void Read();
    void Write();

private:
    // << ---- M E D I U M   L E V E L   I N T E R F A C E ---- >>
    uint Index;
    static double GetPos( uint Index );
    static double SetVel( uint Index, double Vel );
    static double Pos[8];
    static double Vel[8];
    static double Eff[8];
    static double Cmd[8];
    static double Offset[8];
    static bool FreeIndex[8];
    static bool Init();

    // << ---- L O W   L E V E L   I N T E R F A C E ---- >>
    static bool SendPoint();
    static bool GatherInfo();
    static hardware_interface::VelocityJointInterface joint_velocity_interface_;
    static hardware_interface::JointStateInterface    joint_state_interface_;
    static bool KinovaLoaded;
    static double LastSentTime;
    static double LastGatherTime;
    static TrajectoryPoint pointToSend;
    static void * commandLayer_handle;  //Handle for the library's command layer.
    static KinovaDevice devices[MAX_KINOVA_DEVICE];

    // Function pointers to the functions we need
    static int (*MyInitAPI)();
    static int (*MyCloseAPI)();
    static int (*MySendAdvanceTrajectory)(TrajectoryPoint command);
    static int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
    static int (*MyMoveHome)();
    static int (*MyInitFingers)();
    static int (*MyGetAngularCommand)(AngularPosition &);
    static int (*MyEraseAllTrajectories)();
    //static int (*MyGetSensorsInfo)(SensorsInfo &);
    //static int (*MySetActuatorMaxVelocity)(float &);
    //static int (*MyGetActuatorsPosition)(float &);
    //static int (*MyGetAngularVelocity)(float &);
};


#endif //PROJECT_KINOVA_HARDWARE_INTERFACE_H
