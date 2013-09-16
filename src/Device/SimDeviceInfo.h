#ifndef SIMDEVICEINFO_H
#define SIMDEVICEINFO_H

#include <vector>

using namespace std;

// class for every Sensor SimDevice.
class SimDeviceInfo
{
public:
    string           sDeviceName;    // device name (required)
    string           sDeviceType;    // name for device type (required)
    string           sDeviceMode;    // mode of device (optional)
    string           sRobotName;     // name of robot that the device belong to (optional)
    string           sBodyName;      // body that the device may attach to (optional)
    int              m_iFPS;         // fps for cam device (optional)
    vector<string>   m_vSensorList;  // name for sensor of the device (optional)
    vector<string>   m_vModel;       // path of model.xml file for the sensor (optional)
};


#endif // SIMDEVICEINFO_H
