#include <Managers/SimDeviceManager.h>

////////////////////////////////////////////////////////////////////////
/// CONSTRUCTOR
////////////////////////////////////////////////////////////////////////

SimDeviceManager::SimDeviceManager(){

}

////////////////////////////////////////////////////////////////////////
/// INITIALIZERS
////////////////////////////////////////////////////////////////////////

bool SimDeviceManager::Init(PhysicsEngine& rPhysWrapper,
                            GLSceneGraph& rSceneGraph,
                            tinyxml2::XMLDocument& doc,
                            string sProxyName){
  m_Parser.ParseDevices(doc, m_SimDevices, sProxyName);
  cout<<"[SimDeviceManager] Parse all Devices succes!"<<endl;
  m_rPhysWrapper = rPhysWrapper;

  InitDevices(rSceneGraph);
  cout<<"[SimDeviceManager] Init all Devices success!"<<endl;
  return true;
}

////////////////////////////////////////////////////////////////////////


void SimDeviceManager::InitDevices(SceneGraph::GLSceneGraph&  rSceneGraph){
  for(unsigned int i = 0; i!= m_SimDevices.size(); i++){
    SimDeviceInfo Device = m_SimDevices[i];
    string sDeviceType = Device.m_sDeviceType;

    if(sDeviceType == "Camera"){
      InitCamDevice(Device, Device.m_vModel[0], rSceneGraph);
    }

    if(sDeviceType == "GPS"){
      // Nothing here yet...
    }

    if(sDeviceType == "Vicon"){
      InitViconDevice(Device);
    }

    cout<<"get device type "<<sDeviceType<<endl;
    if(sDeviceType == "Controller"){
      InitController(Device);
    }
  }
  cout<<"[SimDeviceManager] init all devices success. "<<endl;
}

////////////////////////////////////////////////////////////////////////

void SimDeviceManager::InitCamDevice(SimDeviceInfo& Device, string sCameraModel,
                                     SceneGraph::GLSceneGraph&  rSceneGraph)
{
  string sCameraName = Device.m_sDeviceName;
  int iFPS = Device.m_iFPS;
  for(unsigned int j = 0; j!= Device.m_vSensorList.size(); j++)
  {
    string sSensorName = Device.m_vSensorList[j]; // e.g. LCameraRGB. This may be bad

    // get init pose for camera via its entity
//    Eigen::Vector6d initPose = m_rPhyMGAgent.m_Agent.GetEntity6Pose(sSensorName);

    Eigen::Vector6d initPose = Device.m_vPose;
    // init sim cam
    cout<<"[SimCam] Camera model use define by RobotURDF is: "<<sCameraModel<<endl;
    SimCam* pSimCam = new SimCam();

    if(sSensorName == "Gray" + sCameraName)           //---------- init gray Cam
    {
      cout<<"[SimDeviceManager] try to init Gray camera, name is "<<sCameraName<<endl;
      pSimCam->init(initPose, sSensorName, eSimCamLuminance, iFPS, sCameraModel, rSceneGraph, m_rPhysWrapper );
    }
    else if(sSensorName == "RGB" + sCameraName)       //---------- init RGB Cam
    {
      cout<<"[SimDeviceManager] try to init RGB camera, name is "<<sSensorName<<endl;
      pSimCam->init(initPose, sSensorName, eSimCamRGB,  iFPS, sCameraModel, rSceneGraph, m_rPhysWrapper );
    }
    else if(sSensorName == "Depth" + sCameraName)     //---------- init Depth Cam
    {
      cout<<"[SimDeviceManager] try to init Depth camera, name is "<<sSensorName<<endl;
      pSimCam->init(initPose,sSensorName,eSimCamLuminance|eSimCamDepth, iFPS, sCameraModel, rSceneGraph, m_rPhysWrapper );
    }

    m_SimCamList.insert(pair<string,SimCam*>(sSensorName,pSimCam));
  }
}

////////////////////////////////////////////////////////////////////////

void SimDeviceManager::InitViconDevice(SimDeviceInfo& Device)
{
  string sDeviceName = Device.m_sDeviceName;
  string sBodyName = Device.m_sBodyName;
  SimVicon* pSimVicon = new SimVicon;
  pSimVicon->init(sDeviceName, sBodyName, m_rPhysWrapper );
  m_SimViconList.insert(pair<string, SimVicon*>(sDeviceName,pSimVicon));
}

////////////////////////////////////////////////////////////////////////

void SimDeviceManager::InitController(SimDeviceInfo& Device)
{
  string sDeviceName = Device.m_sDeviceName;
  string sDeviceMode = Device.m_sDeviceMode;
  string sRobotName = Device.m_sRobotName;

  if(sDeviceMode == "SimpleController")
  {
    SimpleController* pSimpleController = new SimpleController;
    pSimpleController->init(sDeviceName, sRobotName, sDeviceName, m_rPhysWrapper );
    m_SimpleControllerList.insert(pair<string, SimpleController*>(sDeviceName, pSimpleController));
    cout<<"add "<<sDeviceName<<" success "<<endl;
  }

  if(sDeviceMode == "CarController")
  {
    //             CarController* pCarController = new CarController;
  }

}

////////////////////////////////////////////////////////////////////////
/// UPDATE ALL DEVICES
////////////////////////////////////////////////////////////////////////

void SimDeviceManager::UpdateAllDevices()
{
  if(m_SimCamList.size()!=0)
  {
    map<string, SimCam*>::iterator iter;
    for(iter = m_SimCamList.begin();iter!= m_SimCamList.end();iter++)
    {
      iter->second->Update();
    }
  }

  if(m_SimGPSList.size()!=0)
  {
    map<string, SimGPS*>::iterator iter;
    for(iter = m_SimGPSList.begin();iter!= m_SimGPSList.end();iter++)
    {
      iter->second->Update();
    }
  }
  if(m_SimViconList.size()!=0)
  {
    map<string, SimVicon*>::iterator iter;
    for(iter = m_SimViconList.begin();iter!= m_SimViconList.end();iter++)
    {
      iter->second->Update();
    }
  }

}

////////////////////////////////////////////////////////////////////////
/// GET POINTERS TO EVERY DEVICE
////////////////////////////////////////////////////////////////////////

// Get a pointer to the controller

SimpleController* SimDeviceManager::GetSimpleController(string name)
{
  std::map<string, SimpleController*>::iterator iter = m_SimpleControllerList.find(name);
  if(iter == m_SimpleControllerList.end())
  {
    cout<<"[SimDeviceManager] Fatal error! Cannot get device: "<<name<<". Please make sure your robot urdf file has this device!!"<<endl;
  }
  SimpleController* pSimpleController = iter->second;
  return pSimpleController;
}

////////////////////////////////////////////////////////////////////////

SimCam* SimDeviceManager::GetSimCam(string name)
{
  std::map<string, SimCam*>::iterator iter = m_SimCamList.find(name);
  if(iter == m_SimCamList.end())
  {
    cout<<"[SimDeviceManager] Fatal error! Cannot get device: "<<name<<". Please make sure your robot urdf file has this device!!"<<endl;
  }
  SimCam* pSimCam = iter->second;
  return pSimCam;
}

////////////////////////////////////////////////////////////////////////

SimGPS* SimDeviceManager::GetSimGPS(string name)
{
  std::map<string, SimGPS*>::iterator iter =m_SimGPSList.find(name);
  if(iter == m_SimGPSList.end())
  {
    cout<<"[SimDeviceManager] Fatal error! Cannot get device: "<<name<<". Please make sure your robot urdf file has this device!!"<<endl;
  }
  SimGPS* pSimGPS = iter->second;
  return pSimGPS;
}

////////////////////////////////////////////////////////////////////////

SimVicon* SimDeviceManager::GetSimVecon(string name)
{
  std::map<string, SimVicon*>::iterator iter = m_SimViconList.find(name);
  if(iter == m_SimViconList.end())
  {
    cout<<"[SimDeviceManager] Fatal error! Cannot get device: "<<name<<". Please make sure your robot urdf file has this device!!"<<endl;
  }
  SimVicon* pSimVicon = iter->second;
  return pSimVicon;
}


