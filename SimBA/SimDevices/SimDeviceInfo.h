#ifndef SIMDEVICEINFO_H
#define SIMDEVICEINFO_H

#include <vector>
#include <string>
#include <Utils/SE3.h>

using namespace std;

// class for every Sensor SimDevice.
class SimDeviceInfo
{
public:
  SimDeviceInfo(){
    m_bDeviceOn = false;
  }

  string           m_sDeviceName;    // device name
  string           m_sDeviceType;    // name for device type
  string           m_sDeviceMode;
  string           m_sBodyName;
  string           m_sRobotName;
  Eigen::Vector6d  m_vPose;          // init pose of camera
  int              m_iFPS;
  int              m_iBaseline;
  vector<string>   m_vSensorList;    // name for sensor of the device
  vector<string>   m_vModel;         // path of model.xml file for the sensor
  bool             m_bDeviceOn;      // Mark if device is on or not. By default it is off
};


#endif // SIMDEVICEINFO_H
