//
// Created by philippe on 03/05/17.
//

#include "../include/sara_arm/kinova_hardware_interface.h"
#include <dlfcn.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

const uint PERIOD = 5000;

// << ---- S T A T I C   V A R I A B L E   D E C L A R A T I O N ---- >>
bool kinova_hardware_interface::KinovaReady = false;
bool kinova_hardware_interface::KinovaLoaded = false;
double kinova_hardware_interface::LastSentTime = 0;
double kinova_hardware_interface::LastGatherTime = 0;
bool kinova_hardware_interface::FreeIndex[6];
double kinova_hardware_interface::Pos[6];
double kinova_hardware_interface::Vel[6];
double kinova_hardware_interface::Eff[6];
double kinova_hardware_interface::Cmd[6];
double kinova_hardware_interface::Offset[6];
double kinova_hardware_interface::Temperature[6];
hardware_interface::VelocityJointInterface kinova_hardware_interface::joint_velocity_interface_;
hardware_interface::JointStateInterface    kinova_hardware_interface::joint_state_interface_;
TrajectoryPoint kinova_hardware_interface::pointToSend;
void * kinova_hardware_interface::commandLayer_handle;  //Handle for the library's command layer.
KinovaDevice kinova_hardware_interface::devices[MAX_KINOVA_DEVICE];
int (*kinova_hardware_interface::MyInitAPI)();
int (*kinova_hardware_interface::MyCloseAPI)();
int (*kinova_hardware_interface::MySendAdvanceTrajectory)(TrajectoryPoint command);
int (*kinova_hardware_interface::MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int (*kinova_hardware_interface::MyMoveHome)();
int (*kinova_hardware_interface::MyGetSensorsInfo)(SensorsInfo &);
int (*kinova_hardware_interface::MyInitFingers)();
int (*kinova_hardware_interface::MyGetAngularCommand)(AngularPosition &);
int (*kinova_hardware_interface::MyEraseAllTrajectories)();
bool kinova_hardware_interface::TempMonitorOn = false;
bool kinova_hardware_interface::JointTempMonitor[6];
bool kinova_hardware_interface::Simulation = false;
//template <diagnostic_msgs::DiagnosticStatus>;

// << ---- H I G H   L E V E L   I N T E R F A C E ---- >>
kinova_hardware_interface::kinova_hardware_interface( std::string Name, uint Index ) {
    this->Name = Name;
    if ( !KinovaLoaded ){
        KinovaLoaded = true;
        KinovaLoaded = Init();
        KinovaReady = true;
    }
    cmd = 0;
    pos = 0;
    vel = 0;
    eff = 0;
    this->Index = Index;
    FreeIndex[Index] = false;
    hardware_interface::JointStateHandle handle( Name, &pos, &vel, &eff);
    joint_state_interface_.registerHandle(handle);
    joint_velocity_interface_.registerHandle(hardware_interface::JointHandle(handle, &cmd));

    // Do other stuff TODO
}
void kinova_hardware_interface::Read(){
    pos = GetPos( Index );
    if ( TempMonitorOn ) {
        if ( JointTempMonitor[Index] ) {
            if ( Simulation ) {
                Temperature[Index] = ros::Time::now().nsec/1000000000.0;
            }
            diagnostic_msgs::DiagnosticStatus message;
            message.name = Name;
            message.hardware_id = Name;
            diagnostic_msgs::KeyValue KV;
            KV.key = "temperature";
            char chare[50];
            std::sprintf(chare, "%lf", Temperature[Index]);
            KV.value = chare;
            message.values = {  KV  };
            TemperaturePublisher.publish( message );
        }
    }
}
void kinova_hardware_interface::Write(){
    SetVel( Index, cmd );
}
void kinova_hardware_interface::ActivateTemperatureMonitoring( int argc, char **argv ){
    if ( !TempMonitorOn ) {
        TempMonitorOn = true;
    }
    JointTempMonitor[Index] = true;
    std::string NodeName = Name;
    NodeName.append( "_hardware_interface" );
    ros::init(argc, argv, NodeName);
    ros::NodeHandle n;
    TemperaturePublisher = n.advertise<diagnostic_msgs::DiagnosticStatus>( "diagnostic", 100);
    ros::spinOnce();
}

// << ---- M E D I U M   L E V E L   I N T E R F A C E ---- >>
double kinova_hardware_interface::GetPos( uint Index ){
    double Now = ros::Time::now().toNSec();
    bool result;  // true = no error
    if ( LastGatherTime < Now-PERIOD  ){
        //ROS_INFO("Read!");
        LastGatherTime = Now;
        result = GatherInfo();
        if ( !result ) {
            ROS_ERROR("Kinova Hardware Interface.  error detected while trying to gather information");
        }
    }
    return Pos[Index];
}
bool kinova_hardware_interface::SetVel( uint Index, double cmd ){
    double Now =  ros::Time::now().toNSec();
    bool result = true;  // true = no error
    Cmd[Index] = cmd;
    if ( LastSentTime < Now-PERIOD  ){
        //ROS_INFO("Send!");
        LastSentTime = Now;
        result = SendPoint( );
        if ( !result ) {
            ROS_ERROR("Kinova Hardware Interface.  error detected while trying to send point");
        }
    }
    return result;
}

// << ---- L O W   L E V E L   I N T E R F A C E ---- >>
bool kinova_hardware_interface::Init(){
    for ( int i=0; i< 16; i++ ) {
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
        // We load the library

        ROS_INFO("\"* * *            C H A R G E M E N T   D E   L A   B D           * * *\"");
        commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so", RTLD_NOW|RTLD_GLOBAL);

        // We load the functions from the library
        MyInitAPI = (int (*)()) dlsym(commandLayer_handle, "InitAPI");
        MyCloseAPI = (int (*)()) dlsym(commandLayer_handle, "CloseAPI");
        MyMoveHome = (int (*)()) dlsym(commandLayer_handle, "MoveHome");
        MyGetSensorsInfo = (int (*)(SensorsInfo &)) dlsym(commandLayer_handle, "GetSensorsInfo");
        MyEraseAllTrajectories = (int (*)()) dlsym(commandLayer_handle, "EraseAllTrajectories");
        MyInitFingers = (int (*)()) dlsym(commandLayer_handle, "InitFingers");
        MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
        MySendAdvanceTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle, "SendAdvanceTrajectory");
        MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");

    //    MyGetActuatorsPosition = (int (*)(float &)) dlsym(commandLayer_handle, "GetActuatorsPosition");
    //    MyGetAngularVelocity =   (int (*)(float &)) dlsym(commandLayer_handle, "GetAngularVelocity");

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
        if (result != 1 ) {
            if ( nb_attempts > 8 ){
                ROS_INFO("\"* * *                   B R A S   I N T R O U V E                * * *\"");
                ROS_INFO("\"* * *          M O D E   S I M U L A T I O N   A C T I V E       * * *\"");
                Simulation = true;
                Success = true;
            } else {
                ROS_INFO("\"* * *             B R A S   I N T R O U V A B L E                * * *\"");
                ROS_INFO("\"* * *                 T E N T A T I V E   #%d/8                   * * *\"", nb_attempts );
                nb_attempts++;
                sleep(1);
            }
        } else {
            Success = true;
            ROS_INFO("\"* * *      I N I T I A L I S A T I O N   T E R M I N E E         * * *\"");
            ROS_INFO("\"* * *                  B R A S   T R O U V E                     * * *\"");
        }
        //Success = true;
    }

    return true;  // TODO  detect errors
}
bool kinova_hardware_interface::GatherInfo() {

    if ( KinovaReady ) {
        if (Simulation) {
            // TODO simulation
        } else {
            if (TempMonitorOn) {
                SensorsInfo SI;
                MyGetSensorsInfo(SI);
                Temperature[0] = SI.ActuatorTemp1;
                Temperature[1] = SI.ActuatorTemp2;
                Temperature[2] = SI.ActuatorTemp3;
                Temperature[3] = SI.ActuatorTemp4;
                Temperature[4] = SI.ActuatorTemp5;
                Temperature[5] = SI.ActuatorTemp6;
                // TODO publish temperature in a topic
            }
            AngularPosition PositionList;
            MyGetAngularCommand(PositionList);
            Pos[0] = PositionList.Actuators.Actuator1 / 160 * 3.14159 + Offset[0];
            Pos[1] = PositionList.Actuators.Actuator2 / 180 * 3.14159 + Offset[1];
            Pos[2] = PositionList.Actuators.Actuator3 / 180 * 3.14159 + Offset[2];
            Pos[3] = PositionList.Actuators.Actuator4 / 180 * 3.14159 + Offset[3];
            Pos[4] = PositionList.Actuators.Actuator5 / 180 * 3.14159 + Offset[4];
            Pos[5] = PositionList.Actuators.Actuator6 / 180 * 3.14159 + Offset[5];
        }
    }
    return true;  // TODO  detect errors
}
bool kinova_hardware_interface::SendPoint() {

    if ( KinovaReady ) {
        if (Simulation) {
            // TODO simulation
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


