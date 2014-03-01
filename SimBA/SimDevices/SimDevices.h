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
  void AddDevice(SimDeviceInfo devInfo);
  void InitAllDevices();
  void InitCamDevice(SimDeviceInfo& Device, string sCameraModel);
  void InitViconDevice(SimDeviceInfo& Device);
  void InitController(SimDeviceInfo& Device);

  /// Update devices
  void UpdateAllSensors();

  // GETTERS
  SimDeviceInfo GetDeviceInfo(string sDeviceName);
  Controller* GetController(string name);
  SimCamera* GetSimCam(string name);
  SimGPS* GetSimGPS(string name);
  SimVicon* GetSimVecon(string name);

  // MEMBER VARIABLES
  vector<SimDeviceInfo>              m_vSimDevices;
  map<string, SimGPS*>               m_SimGPSList;
  map<string, SimVicon*>             m_SimViconList;
  map<string, SimCamera*>            m_SimCamList;
  map<string, SimLaser2D*>           m_SimLaser2DList;
  map<string, SimLaser3D*>           m_SimLaser3DList;
  map<string, Controller*>           m_ControllerList;

};

#endif // SimDevices_H
