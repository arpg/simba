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
    string           m_sDeviceName;    // device name
    string           m_sDeviceType;    // name for device type
    string           m_sDeviceMode;
    string           m_sBodyName;
    string           m_sRobotName;
    Eigen::Vector6d  m_vPose;          // init pose of camera
    int              m_iFPS;
    vector<string>   m_vSensorList;  // name for sensor of the device
    vector<string>   m_vModel;       // path of model.xml file for the sensor

};


#endif // SIMDEVICEINFO_H
