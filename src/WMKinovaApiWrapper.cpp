// \file WMKinovaApiWrapper.cpp
// \brief Definition of wrapper over Kinova API.
// \author Kevin Blackburn

#include "WMKinovaApiWrapper.h"

#include <inttypes.h>
#include <string>
#include <stdexcept>
#include <dlfcn.h>

using namespace wm_kinova_hardware_interface;

namespace
{
    const std::string KINOVA_LIBRARY_NAME = "Kinova.API.USBCommandLayerUbuntu.so";
}

int (*WMKinovaApiWrapper::MyInitAPI)();
int (*WMKinovaApiWrapper::MyCloseAPI)();
int (*WMKinovaApiWrapper::MySendAdvanceTrajectory)(TrajectoryPoint command);
int (*WMKinovaApiWrapper::MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int (*WMKinovaApiWrapper::MyMoveHome)();
int (*WMKinovaApiWrapper::MyGetSensorsInfo)(SensorsInfo &);
int (*WMKinovaApiWrapper::MyInitFingers)();
int (*WMKinovaApiWrapper::MyGetAngularCommand)(AngularPosition &);
int (*WMKinovaApiWrapper::MyGetAngularForce)(AngularPosition &Response);
int (*WMKinovaApiWrapper::MyEraseAllTrajectories)();

std::atomic_bool WMKinovaApiWrapper::isAPIInitialized(false);

void* WMKinovaApiWrapper::kinovaHandle;

void WMKinovaApiWrapper::initialize() 
{

    if (!isAPIInitialized.load()) 
    {
        dlerror(); // Ensures errors are cleared before attempting to load library.

        kinovaHandle = dlopen(KINOVA_LIBRARY_NAME.c_str(), RTLD_NOW | RTLD_GLOBAL);
        if(kinovaHandle == NULL)
        {
            throw std::runtime_error("Library '" + KINOVA_LIBRARY_NAME + "' couldn't be loaded. Reason: " + dlerror());
        }
        else
        {
            // We load the functions from the library
            MyInitAPI = (int (*)()) dlsym(kinovaHandle, "InitAPI");
            MyCloseAPI = (int (*)()) dlsym(kinovaHandle, "CloseAPI");
            MyMoveHome = (int (*)()) dlsym(kinovaHandle, "MoveHome");
            MyGetSensorsInfo = (int (*)(SensorsInfo &)) dlsym(kinovaHandle, "GetSensorsInfo");
            MyEraseAllTrajectories = (int (*)()) dlsym(kinovaHandle, "EraseAllTrajectories");
            MyInitFingers = (int (*)()) dlsym(kinovaHandle, "InitFingers");
            MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(kinovaHandle,
                                                                                                 "GetDevices");
            MySendAdvanceTrajectory = (int (*)(TrajectoryPoint)) dlsym(kinovaHandle, "SendAdvanceTrajectory");
            MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(kinovaHandle, "GetAngularPosition");
            MyGetAngularForce = (int (*)(AngularPosition &)) dlsym(kinovaHandle, "GetAngularForce");

            
            char* errorDescription = dlerror();
            if (errorDescription != NULL) 
            {
                throw std::runtime_error("An error occurred while loading functions of '" + KINOVA_LIBRARY_NAME + "'. Reason: " + errorDescription);
            }
            else 
            {
                isAPIInitialized = true;
            }
        }
    }
}
