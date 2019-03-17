// \file WMKinovaApiWrapper.h
// \brief Declaration of wrapper over Kinova API.
// Created by kevin on 03/03/2019.

#ifndef PROJECT_WMKINOVAAPIWRAPPER_H
#define PROJECT_WMKINOVAAPIWRAPPER_H

#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"

#include <math.h>
#include <atomic>

namespace wm_kinova_hardware_interface
{

    class WMKinovaApiWrapper final
    {
    public:

        WMKinovaApiWrapper() = delete;

        static void initialize();

        // \brief Checks if API is initialized
        // \return True if API was initialized, false otherwise.
        static bool isInitialized() noexcept { return isAPIInitialized.load(); };

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

        static void* kinovaHandle;
        static std::atomic_bool isAPIInitialized;
    };
} // namespace wm_kinova_hardware_interface
#endif //PROJECT_WMKINOVAAPIWRAPPER_H

