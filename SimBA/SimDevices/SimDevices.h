#ifndef SimDevices_H
#define SimDevices_H

// Info class for cameras
// TODO: Modify? Weird to hold info here.
#include <SimDevices/SimDeviceInfo.h>

// All of our Controllers
#include <SimDevices/Controller/CarController.h>
#include <SimDevices/Controller/SimpleController.h>

// All of our Sensors
#include <SimDevices/Sensor/SimCamera.h>
#include <SimDevices/Sensor/SimGPS.h>
#include <SimDevices/Sensor/SimLaser2D.h>
#include <SimDevices/Sensor/SimLaser3D.h>
#include <SimDevices/Sensor/SimVicon.h>


using namespace std;

class SimDevices
{

public:

  // Constructor
  SimDevices();

  // Initializers
  void AddDevice(SimDeviceInfo* devInfo);

  /// Update devices
  void UpdateSensors();

  // GETTERS
  SimDeviceInfo* GetDeviceInfo(string sDeviceName);

  // MEMBER VARIABLES
  map<string, SimDeviceInfo*>  m_vSimDevices;

};

#endif // SimDevices_H
