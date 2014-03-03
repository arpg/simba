#ifndef SIMDEVICEINFO_H
#define SIMDEVICEINFO_H

#include <vector>
#include <string>
#include <Utils/SE3.h>
#include <ModelGraph/EigenHelpers.h>

using namespace std;

// SimDeviceInfo is a superclass that holds basic info common
// to every SimDevice.

class SimDeviceInfo
{
public:

  // CONSTRUCTOR
  SimDeviceInfo(){
    m_bDeviceOn = false;
  }

  /// SETTERS
  void SetDeviceName(string sDeviceName){
    m_sDeviceName = sDeviceName;
  }

  void SetRobotName(string sRobotName){
    m_sRobotName = sRobotName;
  }

  // Not sure what this does... keeping for now.
  void SetBodyName(string sBodyName){
    m_sBodyName = sBodyName;
  }

  /// GETTERS
  string GetDeviceName(){
    return m_sDeviceName;
  }

  string GetRobotName(){
    return m_sRobotName;
  }

  string GetBodyName(){
    return m_sBodyName;
  }

  // MEMBER VARIABLES
  string           m_sDeviceName;    // Device name (i.e. LCam@RaycastVehicle)
  string           m_sDeviceType;    // Type of device ('Camera')
  string           m_sDeviceMode;    // Mode for device ('RGB')
  string           m_sBodyName;      // Physics body for the device
  string           m_sRobotName;     // Name of the Robot system
  bool             m_bDeviceOn;      // Mark if device is on or not.
  Eigen::Vector6d  m_vPose;

};


#endif // SIMDEVICEINFO_H
