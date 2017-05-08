//
// Created by philippe on 03/05/17.
//

#include "kinova_hardware_interface.h"
#include <dlfcn.h>

// << ---- S T A T I C   V A R I A B L E   D E C L A R A T I O N ---- >>
bool kinova_hardware_interface::KinovaLoaded = false;
double kinova_hardware_interface::LastSentTime = 0;
double kinova_hardware_interface::LastGatherTime = 0;
bool kinova_hardware_interface::FreeIndex[8];
double kinova_hardware_interface::Pos[8];
double kinova_hardware_interface::Vel[8];
double kinova_hardware_interface::Eff[8];
double kinova_hardware_interface::Cmd[8];
double kinova_hardware_interface::Offset[8];
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
int (*kinova_hardware_interface::MyInitFingers)();
int (*kinova_hardware_interface::MyGetAngularCommand)(AngularPosition &);
int (*kinova_hardware_interface::MyEraseAllTrajectories)();

// << ---- H I G H   L E V E L   I N T E R F A C E ---- >>
kinova_hardware_interface::kinova_hardware_interface( std::string Name, uint Index ){
    this->Name = Name;
    if ( !KinovaLoaded ){
        KinovaLoaded = Init();
        KinovaLoaded = true;
    }
    FreeIndex[Index] = false;
    hardware_interface::JointStateHandle handle( Name, &Pos[Index], &Vel[Index], &Eff[Index]);
    joint_state_interface_.registerHandle(handle);
    joint_velocity_interface_.registerHandle(hardware_interface::JointHandle(handle, &Cmd[Index]));

    // Do other stuff TODO
}
void kinova_hardware_interface::Read(){
    Pos[Index] = GetPos( Index );
}
void kinova_hardware_interface::Write(){
    SetVel( Index, Cmd[Index] );
}

// << ---- M E D I U M   L E V E L   I N T E R F A C E ---- >>
double kinova_hardware_interface::GetPos( uint Index ){
    double Now = ros::Time::now().toNSec();
    bool result;  // true = no error
    if ( LastGatherTime < Now-5000  ){
        result = GatherInfo();
        if ( !result ) {
            ROS_ERROR("Kinova Hardware Interface.  error detected while trying to gather information");
        }
        LastGatherTime = Now;
    }
    return Pos[Index];
}
double kinova_hardware_interface::SetVel( uint Index, double cmd ){
    double Now =  ros::Time::now().toNSec();
    bool result = true;  // true = no error
    Cmd[Index] = cmd;
    if ( LastSentTime < Now-5000  ){
        result = SendPoint( );
        if ( !result ) {
            ROS_ERROR("Kinova Hardware Interface.  error detected while trying to send point");
        }
        LastSentTime = Now;
    }
    return result;
}

// << ---- L O W   L E V E L   I N T E R F A C E ---- >>
bool kinova_hardware_interface::Init(){
    for ( int i=0; i< 16; i++ ) {
        FreeIndex[i] = true;
    }
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
            if ( nb_attempts > 9 ){
                ROS_INFO("\"* * *          M O D E   S I M U L A T I O N   A C T I V É       * * *\"");
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
        //Success = true;
    }
    return true;  // TODO  detect errors
}
bool kinova_hardware_interface::GatherInfo() {
    AngularPosition PositionList;
    MyGetAngularCommand(  PositionList );
    Pos[0] = PositionList.Actuators.Actuator1/180*3.14159+Offset[0];
    Pos[1] = PositionList.Actuators.Actuator2/180*3.14159+Offset[1];
    Pos[2] = PositionList.Actuators.Actuator3/180*3.14159+Offset[2];
    Pos[3] = PositionList.Actuators.Actuator4/180*3.14159+Offset[3];
    Pos[4] = PositionList.Actuators.Actuator5/180*3.14159+Offset[4];
    Pos[5] = PositionList.Actuators.Actuator6/180*3.14159+Offset[5];
    Pos[6] = PositionList.Actuators.Actuator5/180*3.14159+Offset[6];
    Pos[7] = PositionList.Actuators.Actuator6/180*3.14159+Offset[7];
    return true;  // TODO  detect errors
}

bool kinova_hardware_interface::SendPoint() {

    //  execute order
    pointToSend.Position.Actuators.Actuator1 = (float) Vel[0];
    pointToSend.Position.Actuators.Actuator2 = (float) Vel[1];
    pointToSend.Position.Actuators.Actuator3 = (float) Vel[2];
    pointToSend.Position.Actuators.Actuator4 = (float) Vel[3];
    pointToSend.Position.Actuators.Actuator5 = (float) Vel[4];
    pointToSend.Position.Actuators.Actuator4 = (float) Vel[5];
    pointToSend.Position.Actuators.Actuator5 = (float) Vel[6];
    pointToSend.Position.Actuators.Actuator4 = (float) Vel[7];
    MyEraseAllTrajectories();
    MySendAdvanceTrajectory(pointToSend);

    return true;  // TODO  detect errors
}