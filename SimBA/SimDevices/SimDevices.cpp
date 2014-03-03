#include <SimDevices/SimDevices.h>

////////////////////////////////////////////////////////////////////////
/// CONSTRUCTOR
////////////////////////////////////////////////////////////////////////

SimDevices::SimDevices(){
}

////////////////////////////////////////////////////////////////////////
/// INITIALIZERS
////////////////////////////////////////////////////////////////////////

void SimDevices::AddDevice(SimDeviceInfo* devInfo){
  m_vSimDevices[devInfo->GetDeviceName()] = devInfo;
}

////////////////////////////////////////////////////////////////////////
/// UPDATE ALL DEVICES
////////////////////////////////////////////////////////////////////////

void SimDevices::UpdateSensors(){
  for(map<string, SimDeviceInfo*>::iterator it = m_vSimDevices.begin();
      it != m_vSimDevices.end();
      it++){
    SimDeviceInfo* Device = it->second;
    if(static_cast<SimCamera*>(Device) != NULL){
      SimCamera* pCam = (SimCamera*) Device;
      pCam->Update();
    }
    else if(static_cast<SimGPS*>(Device) != NULL){
      SimGPS* pGPS = (SimGPS*) Device;
      pGPS->Update();
    }
    else if(static_cast<SimVicon*>(Device) != NULL){
      SimVicon* pVicon = (SimVicon*) Device;
      pVicon->Update();
    }
  }
}


/******************************************************************************
  * GETTERS
  *****************************************************************************/

SimDeviceInfo* SimDevices::GetDeviceInfo(string sDeviceName){
  return m_vSimDevices[sDeviceName];
}

