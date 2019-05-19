
//
// Created by philippe on 03/05/17.
//

#include "WMKinovaHardwareInterface.h"

#include <std_msgs/Float32.h>
#include <iostream>

using namespace wm_kinova_hardware_interface;
using namespace wm_admittance;


// << ---- S T A T I C   V A R I A B L E   I N I T I A L I Z A T I O N ---- >>
bool WMKinovaHardwareInterface::KinovaReady = false;
bool WMKinovaHardwareInterface::KinovaLoaded = false;
double WMKinovaHardwareInterface::LastSentTime = 0;
double WMKinovaHardwareInterface::LastGatherTime = 0;
double WMKinovaHardwareInterface::Current = 0;
double WMKinovaHardwareInterface::Voltage = 0;
bool WMKinovaHardwareInterface::FreeIndex[6]{0};
double WMKinovaHardwareInterface::Pos[6]{0};
double WMKinovaHardwareInterface::Vel[6]{0};
double WMKinovaHardwareInterface::Eff[6]{0};
double WMKinovaHardwareInterface::Cmd[6]{0};
double WMKinovaHardwareInterface::Offset[6];
double WMKinovaHardwareInterface::Temperature[6]{0};
hardware_interface::VelocityJointInterface WMKinovaHardwareInterface::joint_velocity_interface_;
hardware_interface::JointStateInterface    WMKinovaHardwareInterface::joint_state_interface_;
TrajectoryPoint WMKinovaHardwareInterface::pointToSend;
AngularPosition WMKinovaHardwareInterface::ForceList;
AngularPosition WMKinovaHardwareInterface::PositionList;
ros::Publisher WMKinovaHardwareInterface::StatusPublisher;
KinovaDevice WMKinovaHardwareInterface::devices[MAX_KINOVA_DEVICE];

WMAdmittance* WMKinovaHardwareInterface::aAdmittance;

bool WMKinovaHardwareInterface::StatusMonitorOn = true;
bool WMKinovaHardwareInterface::Simulation = false;

IndexByJointNameMapType WMKinovaHardwareInterface::aIndexByJointNameMap;

// << ---- H I G H   L E V E L   I N T E R F A C E ---- >>

WMKinovaHardwareInterface::WMKinovaHardwareInterface(){

}

std::thread WMKinovaHardwareInterface::tread(&WMKinovaHardwareInterface::run);
std::mutex WMKinovaHardwareInterface::treadMutex;
bool WMKinovaHardwareInterface::stillSending{false};
bool WMKinovaHardwareInterface::stillGettingPosition{false};
bool WMKinovaHardwareInterface::stillGettingTorque{false};


bool WMKinovaHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{
    if (!KinovaLoaded)
    {
        KinovaLoaded = true;
        KinovaLoaded = InitKinova();
        KinovaReady = true;
    }

    RetrieveDevices();

    for (int i = 0; i < 16; i++)
    {
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

    Name = "";
    Index = 0;
    std::vector<std::string> Joints;

    // Mandatory parameters
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

    aIndexByJointNameMap.emplace(Index, Name);

    if (!robot_hw_nh.getParam("complience_level", ComplienceLevel)){
        ComplienceLevel = 1;
    }
    if (!robot_hw_nh.getParam("complience_threshold", ComplienceThreshold)){
        ComplienceThreshold = 100;
    }
    if (!robot_hw_nh.getParam("complience_derivation_factor", ComplienceDerivationFactor)){
        ComplienceThreshold = 0.01;
    }
    if (!robot_hw_nh.getParam("complience_loss_factor", ComplienceLossFactor)){
        ComplienceThreshold = 0.75;
    }
    if (!robot_hw_nh.getParam("complience_resistance", ComplienceResistance)){
        ComplienceResistance = 0.5;
    }
    if (!robot_hw_nh.getParam("speed_ratio", SpeedRatio)){
        SpeedRatio = 1;
    }

    aAdmittance = wm_admittance::WMAdmittance::getInstance();

    cmd = 0;
    pos = 0;
    vel = 0;
    eff = 0;
    FreeIndex[Index] = false;

    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(Name, &pos, &vel, &eff));
    joint_velocity_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(Name), &cmd));
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_velocity_interface_);

    //TemperaturePublisher = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 100);

    treadMutex.lock();
    seff = Eff[Index];
    treadMutex.unlock();
    deff = seff;
    return true;
}

void WMKinovaHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {

    GetInfos();
    //std::cout << "\nIndex = " << Index << ", Position = " << Pos[Index] << ", Effort = " << Eff[Index];
    diagnostic_msgs::DiagnosticArray dia_array;

    treadMutex.lock();
    pos = AngleProxy( 0, Pos[Index]);
    eff = Eff[Index];
    vel = Vel[Index];
    treadMutex.unlock();

//    diagnostic_msgs::DiagnosticStatus dia_status;
//    dia_status.name = "kinova_arm";
//    dia_status.hardware_id = Name;
//
//    diagnostic_msgs::KeyValue KV1;
//    KV1.key = "temperature";
//    char chare[50];
//    std::sprintf(chare, "%lf", Temperature[Index]);
//    KV1.value = chare;
//
//    diagnostic_msgs::KeyValue KV2;
//    KV2.key = "torque";
//    std::sprintf(chare, "%lf", Eff[Index]);
//    KV2.value = chare;
//
//    dia_status.values.push_back(KV1);
//    dia_status.values.push_back(KV2);
//
//    dia_array.status.push_back(dia_status);
//
//    TemperaturePublisher.publish(dia_array);
}

void WMKinovaHardwareInterface::write(const ros::Time &time, const ros::Duration &period)
{
    double cmdVel{cmd*9.549296586}; // rad/s to RPM
    if (aAdmittance->isAdmittanceEnabled()) {
        cmdVel += aAdmittance->getAdmittanceVelocityFromJoint(aIndexByJointNameMap[Index]);
    }
    else {
        seff += (eff-seff)*ComplienceLossFactor;
        deff += (seff-deff)*ComplienceDerivationFactor;

        if ((seff-deff)*(seff-deff)>ComplienceThreshold) {
            cmdVel += (-2*seff+deff)*ComplienceLevel;
        }
        else {
            deff += (seff-deff)*ComplienceResistance;
        }
    }

    SetVel(Index, cmdVel*SpeedRatio); // from r/s to ded/p

}

// << ---- M E D I U M   L E V E L   I N T E R F A C E ---- >>
bool WMKinovaHardwareInterface::GetInfos() {

    return true;
}

bool WMKinovaHardwareInterface::SetVel(int Index, double cmd) {

    treadMutex.lock();
    Cmd[Index] = cmd;
    treadMutex.unlock();
    return true;
}

// << ---- L O W   L E V E L   I N T E R F A C E ---- >>
bool WMKinovaHardwareInterface::InitKinova() noexcept
{
    bool kinovaInitialized = true;

    ROS_INFO("\"* * *            C H A R G E M E N T   D E   K I N O V A   A P I           * * *\"");
    try
    {
        if (!WMKinovaApiWrapper::isInitialized())
        {
            WMKinovaApiWrapper::initialize(); // Can throw
        }
    }
    catch (const std::exception& exception)
    {
        kinovaInitialized = false;
        ROS_ERROR("Exception was raised when attempting to initialize kinova API.");
        ROS_ERROR("Reason: %s", exception.what());
    }
    return kinovaInitialized;
}

bool WMKinovaHardwareInterface::RetrieveDevices()
{

    bool Success = false;
    int nb_attempts = 2;
    int result;
    ROS_INFO("\"* * *              R E C H E R C H E   D U   B R A S             * * *\"");
    while (!Success) {

        result = (*WMKinovaApiWrapper::MyInitAPI)();
        WMKinovaApiWrapper::MyGetDevices(devices, result);
        if (result != 1) {
            if (nb_attempts > 4) {
                ROS_INFO("\"* * *                   B R A S   I N T R O U V E                * * *\"");
                ROS_INFO("\"* * *          M O D E   S I M U L A T I O N   A C T I V E       * * *\"");
                Simulation = true;
                Success = true;
            } else {
                ROS_INFO("\"* * *             B R A S   I N T R O U V A B L E                * * *\"");
                ROS_INFO("\"* * *                 T E N T A T I V E   #%d/3                   * * *\"",
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
    StatusPublisher = n.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 100);
    ros::spinOnce();
    return true;
}

void *WMKinovaHardwareInterface::SendToKinova(){
    try {
        WMKinovaApiWrapper::MySendAdvanceTrajectory(pointToSend);
    } catch(...) {
        ROS_ERROR("Unable to send command to kinova arm");
    }
    stillSending = false;
}

void *WMKinovaHardwareInterface::GetTorqueFromKinova(){
    try {
        WMKinovaApiWrapper::MyGetAngularForce(ForceList);
    } catch(...) {
        ROS_ERROR("Unable to get torque from kinova arm");
    }
    stillGettingTorque = false;
}

void *WMKinovaHardwareInterface::GetpositionFromKinova(){
    try {
        WMKinovaApiWrapper::MyGetAngularCommand(PositionList);
    } catch(...) {
        ROS_ERROR("Unable to get position from kinova arm");
    }
    stillGettingPosition = false;
}

bool WMKinovaHardwareInterface::GatherInfo() {

    if (KinovaReady) {
        if (Simulation) {
            // Do crude simulation
            for (int i = 0; i < 6; i++) {
                Temperature[i] = 0.1234;
                Pos[i] += Vel[i] / 50000;
                Eff[i] += 1;
            }
            Current = -1;
            Voltage = -1;
        } else {

            if (stillGettingPosition){
                ROS_ERROR("Getting position from kinova took too long.");
            } else {

                treadMutex.lock();
                Pos[0] = PositionList.Actuators.Actuator1 / 180 * M_PI - Offset[0];
                Pos[1] = PositionList.Actuators.Actuator2 / 180 * M_PI - Offset[1];
                Pos[2] = PositionList.Actuators.Actuator3 / 180 * M_PI - Offset[2];
                Pos[3] = PositionList.Actuators.Actuator4 / 180 * M_PI - Offset[3];
                Pos[4] = PositionList.Actuators.Actuator5 / 180 * M_PI - Offset[4];
                Pos[5] = PositionList.Actuators.Actuator6 / 180 * M_PI - Offset[5];
                treadMutex.unlock();

                stillGettingPosition = true;
                std::thread getterPos(&WMKinovaHardwareInterface::GetpositionFromKinova);
                getterPos.detach();

            }

            if (stillGettingTorque){
                ROS_ERROR("Getting torque from kinova took too long.");
            } else {

                treadMutex.lock();
                Eff[0] = ForceList.Actuators.Actuator1;
                Eff[1] = ForceList.Actuators.Actuator2;
                Eff[2] = ForceList.Actuators.Actuator3;
                Eff[3] = ForceList.Actuators.Actuator4;
                Eff[4] = ForceList.Actuators.Actuator5;
                Eff[5] = ForceList.Actuators.Actuator6;
                treadMutex.unlock();

                stillGettingTorque = true;
                std::thread getterPos(&WMKinovaHardwareInterface::GetTorqueFromKinova);
                getterPos.detach();

            }


//
//            SensorsInfo SI;
//            WMKinovaApiWrapper::MyGetSensorsInfo(SI);
//            Temperature[0] = SI.ActuatorTemp1;
//            Temperature[1] = SI.ActuatorTemp2;
//            Temperature[2] = SI.ActuatorTemp3;
//            Temperature[3] = SI.ActuatorTemp4;
//            Temperature[4] = SI.ActuatorTemp5;
//            Temperature[5] = SI.ActuatorTemp6;
//            Current = SI.Current;
//            Voltage = SI.Voltage;

        }

    }
    return true;  // TODO  detect errors
}

bool WMKinovaHardwareInterface::SendPoint() {

    if (KinovaReady) {
        treadMutex.lock();
        for (int i = 0; i < 6; i++) {
            // Apply hardcoded speed limits
            Cmd[i] = Cmd[i] < 40 ? Cmd[i] : 40;
            Cmd[i] = Cmd[i] > -40 ? Cmd[i] : -40;
            Vel[i] = Cmd[i];
        }
        treadMutex.unlock();
        if (Simulation) {
            // Do crude simulation
            treadMutex.lock();
            for (int i = 0; i < 6; i++) {
                Temperature[i] = 0;
            }
            treadMutex.unlock();
        } else {

            if (stillSending){
                ROS_ERROR("Sending to kinova took too long.");
            } else {
                //  execute order
                treadMutex.lock();
                pointToSend.Position.Actuators.Actuator1 = (float) Cmd[0];
                pointToSend.Position.Actuators.Actuator2 = (float) Cmd[1];
                pointToSend.Position.Actuators.Actuator3 = (float) Cmd[2];
                pointToSend.Position.Actuators.Actuator4 = (float) Cmd[3];
                pointToSend.Position.Actuators.Actuator5 = (float) Cmd[4];
                pointToSend.Position.Actuators.Actuator6 = (float) Cmd[5];
                treadMutex.unlock();

                stillSending = true;
                std::thread sender(&WMKinovaHardwareInterface::SendToKinova);
                sender.detach();

            }
//            WMKinovaApiWrapper::MySendAdvanceTrajectory(pointToSend);

        }
    }
    return true;  // TODO  detect errors
}

void *WMKinovaHardwareInterface::run() {
    ros::Rate rate(100);
    int i{0};
    while (ros::ok()){
        try {
            if ((i++ % 5) == 0) {
                GatherInfo();
            }

            SendPoint();
        } catch ( ... ){
            ROS_ERROR("bad communication with kinova. Check thread.");
        }

        rate.sleep();
    }

    ROS_ERROR("The kinova thread died.");
}


PLUGINLIB_EXPORT_CLASS( wm_kinova_hardware_interface::WMKinovaHardwareInterface, hardware_interface::RobotHW)
