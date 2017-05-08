#ifndef ARM_H_I
#define ARM_H_I

#include <Kinova.API.CommLayerUbuntu.h>
#include <KinovaTypes.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <string>

int NOMBRE_DE_MOTEURS_KINOVA = 0;
//Handle for the library's command layer.
void * commandLayer_handle;
int result;  // Tampon de réception des résultats de certaines fonctions
int devicesCount;
KinovaDevice devices[MAX_KINOVA_DEVICE];

// Function pointers to the functions we need
int (*MyInitAPI)();
int (*MyCloseAPI)();
int (*MySendBasicTrajectory)(TrajectoryPoint command);
int (*MySendAdvanceTrajectory)(TrajectoryPoint command);
int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int (*MySetActiveDevice)(KinovaDevice device);
int (*MyMoveHome)();
int (*MySetJointZero)(int ActuatorAdress);
int (*MyInitFingers)();
int (*MyGetAngularCommand)(AngularPosition &);
int (*MyEraseAllTrajectories)();
int (*MyGetSensorsInfo)(SensorsInfo &);
int (*MySetActuatorMaxVelocity)(float &);
int (*MyGetActuatorsPosition)(float &);
int (*MyGetAngularVelocity)(float &);



std::vector<double> Offsets;
std::vector<std::string> Names;


class ArmHardware : public hardware_interface::RobotHW {
    public:
        ArmHardware();
        void init();
        void Read();
        void Write();
        //double Offset[8];
    private:
        hardware_interface::VelocityJointInterface joint_velocity_interface_;
        hardware_interface::JointStateInterface    joint_state_interface_;
        hardware_interface::JointStateHandle       joint_state_Handle_;

        // Ros Control variables
        double cmd[8];
        double pos[8];
        double vel[8];
        double eff[8];

        //Test variables
        double MaPos[8];
        double MaVel[8];
        TrajectoryPoint pointToSend;

};

#endif