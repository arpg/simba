#ifndef SIMDEVICEMANAGER_H
#define SIMDEVICEMANAGER_H

#include <SimDevices/SimDeviceInfo.h>
#include <SimDevices/SimDevices.h>

using namespace std;

class SimDeviceManager
{

public:
  vector<SimDeviceInfo>              m_vSimDevices;

  /// <DeviceName, SimGPS*>
  map<string, SimGPS*>               m_SimGPSList;
  map<string, SimVicon*>             m_SimViconList;
  map<string, SimCamera*>            m_SimCamList;
  map<string, SimLaser2D*>           m_SimLaser2DList;
  map<string, SimLaser3D*>           m_SimLaser3DList;
  map<string, SimpleController*>     m_SimpleControllerList;
  map<string, CarController*>        m_CarControllerList;
  PoseController                     m_SimpPoseController;
  ModelGraphBuilder*                 m_pModelGraph;
  string                             m_sServerOption;

  /// Constructor
  SimDeviceManager(ModelGraphBuilder* pScene);

  /// Initializers
  void AddDevice(SimDeviceInfo devInfo);
  void InitAllDevices(string sServerOption);
  void InitDeviceByName(string sDeviceName);
  void InitCamDevice(SimDeviceInfo& Device, string sCameraModel);
  void InitViconDevice(SimDeviceInfo& Device);
  void InitController(SimDeviceInfo& Device);

  /// Update devices
  void UpdateAllDevices();

  SimDeviceInfo GetDeviceInfo(string sDeviceName);

  /// Get pointers to all devices
  SimpleController* GetSimpleController(string name);
  SimCamera* GetSimCam(string name);
  SimGPS* GetSimGPS(string name);
  SimVicon* GetSimVecon(string name);
};

#endif // SIMDEVICEMANAGER_H
