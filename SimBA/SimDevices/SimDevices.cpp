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
  m_vSimDevices.push_back(devInfo);
}

////////////////////////////////////////////////////////////////////////
void SimDevices::InitAllDevices(){
  for(unsigned int i = 0; i!= m_vSimDevices.size(); i++){
    SimDeviceInfo& rDevice = m_vSimDevices[i];
    string sDeviceType = rDevice.m_sDeviceType;
    cout<<"[InitAllDevices] Producing device type "<<sDeviceType<<endl;
    if(sDeviceType == "Camera"){
      InitCamDevice(rDevice, rDevice.m_vModel[0]);
    }
    if(sDeviceType == "GPS"){
      // Nothing here yet...
    }
    if(sDeviceType == "Vicon"){
      InitViconDevice(rDevice);
    }
    if(sDeviceType == "Controller"){
      InitController(rDevice);
    }
    // TODO: Why does this matter? I never see it used.
    rDevice.m_bDeviceOn = true;
    cout<<"[SimDevices] The device has been activated!"<<endl;
  }
  cout<<"[SimDevices] Successfully init all devices. "<<endl;
}

//////////////////////////////////////////////////////////////////////////

void SimDevices::InitCamDevice(SimDeviceInfo& Device, string sCameraModel){

  for(unsigned int j = 0; j!= Device.m_vSensorList.size(); j++){
    string sSensorName = Device.m_vSensorList[j];

    Eigen::Vector6d initPose = Device.m_vPose;
    // init sim cam
    cout<<"[SimCamera] Camera model use define by RobotURDF is: "<<
          sCameraModel<<endl;
    SimCamera* pSimCam = new SimCamera();

    // Init gray Cam
    if(sSensorName == "Grey"+sCameraName){
      cout<<"[SimDevices] try to init Gray camera, name is "<<
            sCameraName<<endl;
      pSimCam->init(initPose, sCameraName, SceneGraph::eSimCamLuminance,
                    iFPS, sCameraModel);
    }
    // Init RGB Cam
    else if(sSensorName == "RGB"+sCameraName){
      cout<<"[SimDevices] try to init RGB camera, name is "<<
            sCameraName<<endl;
      pSimCam->init(initPose, sCameraName, SceneGraph::eSimCamRGB,
                    iFPS, sCameraModel);
    }
    // Init Depth Cam
    else if(sSensorName == "Depth"+sCameraName){
      cout<<"[SimDevices] try to init Depth camera, name is "<<
            sCameraName<<endl;
      pSimCam->init(initPose, sCameraName,
                    /*SceneGraph::eSimCamLuminance | */SceneGraph::eSimCamDepth,
                    iFPS, sCameraModel);
    }

    m_SimCamList.insert(pair<string,SimCamera*>(sSensorName,pSimCam));
  }
}

////////////////////////////////////////////////////////////////////////

void SimDevices::InitViconDevice(SimDeviceInfo& Device){
  string sDeviceName = Device.m_sDeviceName;
  string sBodyName = Device.m_sBodyName;
  SimVicon* pSimVicon = new SimVicon;
  pSimVicon->init(sDeviceName, sBodyName);
  Device.m_bDeviceOn = true;
  m_SimViconList.insert(pair<string, SimVicon*>(sDeviceName,pSimVicon));
}

////////////////////////////////////////////////////////////////////////

void SimDevices::InitController(SimDeviceInfo& Device){
  string sDeviceMode = Device.m_sDeviceMode;
  string sDeviceName = Device.m_sDeviceName;
  string sRobotName = Device.m_sRobotName;
  if(sDeviceMode == "SimpleController"){
    SimpleController* pSimpleController =
        new SimpleController(sDeviceName, sRobotName, sDeviceName);
    Device.m_bDeviceOn = true;
    m_ControllerList[sDeviceName] = pSimpleController;
  }
  if(sDeviceMode == "CarController"){
    CarController* pCarController =
        new CarController(sDeviceName, sRobotName, sDeviceName);
    Device.m_bDeviceOn = true;
    m_ControllerList[sDeviceName] = pCarController;
  }
  cout<<"[InitController] Successfully initalized "<<sDeviceName<<endl;
}

////////////////////////////////////////////////////////////////////////
/// UPDATE ALL DEVICES
////////////////////////////////////////////////////////////////////////

void SimDevices::UpdateAllSensors(){
  for(unsigned int i = 0; i!= m_vSimDevices.size();i++){
    SimDeviceInfo Device = m_vSimDevices[i];
    if(Device.m_bDeviceOn==true){
      if(Device.m_sDeviceType =="Camera" ){
        if(m_SimCamList.size()!=0){
          map<string, SimCamera*>::iterator iter;
          for(iter = m_SimCamList.begin();iter!= m_SimCamList.end();iter++){
            iter->second->Update();
          }
        }
      }
      else if(Device.m_sDeviceType == "GPS"){
        if(m_SimGPSList.size()!=0){
          map<string, SimGPS*>::iterator iter;
          for(iter = m_SimGPSList.begin();iter!= m_SimGPSList.end();iter++){
            iter->second->Update();
          }
        }
      }
      else if(Device.m_sDeviceType == "Vicon"){
        if(m_SimViconList.size()!=0){
          map<string, SimVicon*>::iterator iter;
          for(iter = m_SimViconList.begin();
              iter!= m_SimViconList.end();iter++){
            iter->second->Update();
          }
        }
      }
    }
  }
}



/******************************************************************************
  * GETTERS
  *****************************************************************************/

SimDeviceInfo SimDevices::GetDeviceInfo(string sDeviceName){
  for(unsigned int i=0;i!=m_vSimDevices.size(); i++){
    SimDeviceInfo& Device = m_vSimDevices[i];
    if(sDeviceName == Device.m_sDeviceName ){
      return m_vSimDevices[i];
    }
  }
  cout<<"[SimDevices] Fatal Error, Cannot get device info with name "<<
        sDeviceName<<endl;
  exit(-1);
}

////////////////

Controller* SimDevices::GetController(string name){
  Controller* pController = m_ControllerList[name];
  return pController;
}

////////////////

SimCamera* SimDevices::GetSimCam(string name){
  std::map<string, SimCamera*>::iterator iter = m_SimCamList.find(name);
  if(iter == m_SimCamList.end()){
    cout<<"[SimDevices] Fatal error! Cannot get device: "<<name<<
          ". Please make sure your robot urdf file has this device!!"<<endl;
  }
  SimCamera* pSimCam = iter->second;
  return pSimCam;
}

////////////////

SimGPS* SimDevices::GetSimGPS(string name){
  std::map<string, SimGPS*>::iterator iter =m_SimGPSList.find(name);
  if(iter == m_SimGPSList.end()){
    cout<<"[SimDevices] Fatal error! Cannot get device: "<<name<<
          ". Please make sure your robot urdf file has this device!!"<<endl;
  }
  SimGPS* pSimGPS = iter->second;
  return pSimGPS;
}

////////////////

SimVicon* SimDevices::GetSimVecon(string name){
  std::map<string, SimVicon*>::iterator iter = m_SimViconList.find(name);
  if(iter == m_SimViconList.end()){
    cout<<"[SimDevices] Fatal error! Cannot get device: "<<name<<
          ". Please make sure your robot urdf file has this device!!"<<endl;
  }
  SimVicon* pSimVicon = iter->second;
  return pSimVicon;
}
