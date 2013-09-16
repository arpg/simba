#ifndef SIMDEVICEINFO_H
#define SIMDEVICEINFO_H

#include <vector>

using namespace std;

// class for every Sensor SimDevice.
class SimDeviceInfo
{
public:
    string           sDeviceName;    // device name
    string           sDeviceType;    // name for device type
    string           sBodyName;
    vector<string>   m_vSensorList;  // name for sensor of the device
    vector<string>   m_vModel;       // path of model.xml file for the sensor
};


#endif // SIMDEVICEINFO_H
