#ifndef CARCONTROLLER_H
#define CARCONTROLLER_H

#include "SimDevices/SimDeviceInfo.h"
#include "PB_Headers/SimMessages.pb.h"

class CarController: public SimDeviceInfo
{
public:

  CarController(string sDeviceName, string sRobotName){
    SetDeviceName(sDeviceName);
    SetRobotName(sRobotName);
  }

  void UpdateCommand(VehicleMsg& Command){
    m_dSteering = Command.steering_angle();
    m_dTorque = Command.desired_force();
  }

  double m_dSteering;
  double m_dTorque;
};

#endif // CARCONTROLLER_H
