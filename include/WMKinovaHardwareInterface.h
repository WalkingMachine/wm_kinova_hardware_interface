//
// Created by philippe on 03/05/17.
//

#ifndef PROJECT_WMKinovaHardwareInterface_H
#define PROJECT_WMKinovaHardwareInterface_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <string>
#include <ros/ros.h>
#include <WMKinovaHardwareInterface.h>
#include <hardware_interface/joint_command_interface.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/KeyValue.h"

#include "WMKinovaApiWrapper.h"

#include <pluginlib/class_list_macros.h>
#include <math.h>


namespace wm_kinova_hardware_interface
{

    class WMKinovaHardwareInterface : public hardware_interface::RobotHW {
    public:
        // << ---- H I G H   L E V E L   I N T E R F A C E ---- >>
        // Functions
        virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);
        virtual void read(const ros::Time &time, const ros::Duration &period);
        virtual void write(const ros::Time &time, const ros::Duration &period);
        static bool StartStatusMonitoring(int argc, char **argv);

        // Variables
        std::string Name;
        double cmd;
        double pos;
        double vel;
        double eff;
        double seff;
        double deff;
        double SpeedRatio;
        double ComplienceLevel;
        double ComplienceThreshold;
        double ComplienceLossFactor;
        double ComplienceDerivationFactor;
        double ComplienceResistance;


        static double Cmd[6];
        static double Pos[6];
        static double Vel[6];
        static double Eff[6];
        static double Temperature[6];
        static double Offset[6];

    private:
        // << ---- M E D I U M   L E V E L   I N T E R F A C E ---- >>
        // Functions
        static bool GetInfos();
        static bool SetVel(int Index, double Vel);
        static bool InitKinova() noexcept;
        static bool RetrieveDevices();

        // Variables
        int Index;
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
        static hardware_interface::JointStateInterface joint_state_interface_;
        static bool KinovaReady;
        static bool KinovaLoaded;
        static double LastSentTime;
        static double LastGatherTime;
        static TrajectoryPoint pointToSend;
        static KinovaDevice devices[MAX_KINOVA_DEVICE];
    };

    inline double Mod( double A, double N ) {
        return A-floor(A/N)*N;
    }

    inline double AngleProxy( double A1 = 0, double A2 = 0 ) {  // Give the smallest difference between two angles in rad
        A1 = A2-A1;
        A1 = Mod( A1+M_PI, 2*M_PI )-M_PI;
        return A1;
    }
}
#endif //PROJECT_WMKinovaHardwareInterface_H

