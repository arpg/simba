#include <Managers/SimDeviceManager.h>

////////////////////////////////////////////////////////////////////////
/// CONSTRUCTOR
////////////////////////////////////////////////////////////////////////

SimDeviceManager::SimDeviceManager(ModelGraphBuilder *pScene){
  m_pModelGraph = pScene;
}

////////////////////////////////////////////////////////////////////////
/// INITIALIZERS
////////////////////////////////////////////////////////////////////////

void SimDeviceManager::AddDevice(SimDeviceInfo devInfo){
  m_vSimDevices.push_back(devInfo);
}

////////////////////////////////////////////////////////////////////////
void SimDeviceManager::InitAllDevices(string sServerOption)
{
  m_sServerOption = sServerOption;
  if(m_sServerOption == "WithoutNetwork")
  {
    for(unsigned int i = 0; i!= m_vSimDevices.size(); i++)
    {
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

      rDevice.m_bDeviceOn = true;
      cout<<"[SimDeviceManager] trun device to on"<<endl;
    }

    cout<<"[SimDeviceManager] Successfully init all devices. "<<endl;
  }
  else
  {
    cout<<"[SimDeviceManager] Skip. Wait for HAL call for  init devices"<<endl;
  }
}


////////////////////////////////////////////////////////////////////////
void SimDeviceManager::InitDeviceByName(string sDeviceName)
{
  SimDeviceInfo Device = GetDeviceInfo(sDeviceName);
  string sDeviceType = Device.m_sDeviceType;

  if(sDeviceType == "Camera"){
    InitCamDevice(Device, Device.m_vModel[0]);
  }

  if(sDeviceType == "GPS"){
  }

  if(sDeviceType == "Vicon"){
    InitViconDevice(Device);
  }
}

////////////////////////////////////////////////////////////////////////

void SimDeviceManager::InitCamDevice(SimDeviceInfo& Device, string sCameraModel)
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
    cout<<"[SimCamera] Camera model use define by RobotURDF is: "<<sCameraModel<<endl;
    SimCamera* pSimCam = new SimCamera();

    if(sSensorName == "Grey"+sCameraName)           //---------- init gray Cam
    {
      cout<<"[SimDeviceManager] try to init Gray camera, name is "<<sCameraName<<endl;
      pSimCam->init(initPose, sCameraName, eSimCamLuminance,
                    iFPS, sCameraModel, m_pModelGraph );
    }
    else if(sSensorName == "RGB"+sCameraName)       //---------- init RGB Cam
    {
      cout<<"[SimDeviceManager] try to init RGB camera, name is "<<sCameraName<<endl;
      pSimCam->init(initPose, sCameraName, eSimCamRGB,
                    iFPS, sCameraModel, m_pModelGraph );
    }
    else if(sSensorName == "Depth"+sCameraName)     //---------- init Depth Cam
    {
      cout<<"[SimDeviceManager] try to init Depth camera, name is "<<sCameraName<<endl;
      pSimCam->init(initPose, sCameraName, eSimCamLuminance | eSimCamDepth,
                    iFPS, sCameraModel, m_pModelGraph );
    }

    m_SimCamList.insert(pair<string,SimCamera*>(sSensorName,pSimCam));
  }
}

////////////////////////////////////////////////////////////////////////

void SimDeviceManager::InitViconDevice(SimDeviceInfo& Device)
{
  string sDeviceName = Device.m_sDeviceName;
  string sBodyName = Device.m_sBodyName;
  SimVicon* pSimVicon = new SimVicon;
  pSimVicon->init(sDeviceName, sBodyName, m_pModelGraph->m_Phys );
  Device.m_bDeviceOn = true;
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
    pSimpleController->init(sDeviceName, sRobotName,
                            sDeviceName, m_pModelGraph->m_Phys );
    Device.m_bDeviceOn = true;
    m_SimpleControllerList.insert(pair<string,
                                  SimpleController*>(sDeviceName,
                                                     pSimpleController));
    cout<<"[InitController] Successfully init "<<sDeviceName<<endl;
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
  if(m_sServerOption == "WithoutNetwork")
  {
    for(unsigned int i = 0; i!= m_vSimDevices.size();i++)
    {
      SimDeviceInfo Device = m_vSimDevices[i];

      if(Device.m_bDeviceOn==true)
      {
        if(Device.m_sDeviceType =="Camera" )
        {
          if(m_SimCamList.size()!=0)
            {
              map<string, SimCamera*>::iterator iter;
              for(iter = m_SimCamList.begin();iter!= m_SimCamList.end();iter++)
              {
                iter->second->Update();
              }
            }
        }
        else if(Device.m_sDeviceType == "GPS")
        {
          if(m_SimGPSList.size()!=0)
          {
            map<string, SimGPS*>::iterator iter;
            for(iter = m_SimGPSList.begin();iter!= m_SimGPSList.end();iter++)
            {
              iter->second->Update();
            }
          }
        }
        else if(Device.m_sDeviceType == "Vicon")
        {

          if(m_SimViconList.size()!=0)
          {
            map<string, SimVicon*>::iterator iter;
            for(iter = m_SimViconList.begin();iter!= m_SimViconList.end();iter++)
            {
              iter->second->Update();
            }
          }
        }
      }
      else
      {
        cout<<"skip update device"<<endl;
      }
    }
  }
}

SimDeviceInfo SimDeviceManager::GetDeviceInfo(string sDeviceName)
{
  for(unsigned int i=0;i!=m_vSimDevices.size(); i++)
  {
    SimDeviceInfo& Device = m_vSimDevices[i];
    if(sDeviceName == Device.m_sDeviceName )
    {
      return m_vSimDevices[i];
    }
  }

  cout<<"[SimDeviceManager] Fatal Error, Cannot get device info with name "<<sDeviceName<<endl;
  exit(-1);
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

SimCamera* SimDeviceManager::GetSimCam(string name)
{
  std::map<string, SimCamera*>::iterator iter = m_SimCamList.find(name);
  if(iter == m_SimCamList.end())
  {
    cout<<"[SimDeviceManager] Fatal error! Cannot get device: "<<name<<". Please make sure your robot urdf file has this device!!"<<endl;
  }
  SimCamera* pSimCam = iter->second;
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
