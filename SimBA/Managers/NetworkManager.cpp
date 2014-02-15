#include <Managers/NetworkManager.h>


////////////////////////////////////////////////////////////////////////
/// INITIALIZE NODE NETWORK AND ALL DEVICES
////////////////////////////////////////////////////////////////////////
// The following try to init Node for Network Manager
bool NetworkManager::Init( string sProxyName, string sServerName, int verbocity)
{
  // 1, check if we need to init Node for RobotProxy
  m_sServerName = sServerName;
  if(m_sServerName == "WithoutNetwork")
  {
    return true;
  }

  // 2. init node of Robot Proxy and provide relative rpc method for Robot code
  m_sLocalSimName  = sProxyName;
  m_verbocity      = verbocity;
  m_SubscribeNum   = 0;

  m_Node.set_verbocity(m_verbocity ); // make some noise on errors
  if( m_Node.init(m_sLocalSimName)== true)
  {
    cout<<"[NetworkManager] Init Proxy Node '"<<m_sLocalSimName<<"' success!"<<endl;
    return true;
  }
  else
  {
    cout<<"[NetworkManager] Init Proxy Node '"<<m_sLocalSimName<<"' Fail !"<<endl;
    return false;
  }
}


////////////////////////////////////////////////////////////////////////
// The following is only used when we need to sync robot state with statekeeper.
bool NetworkManager::PubRobotIfNeeded(RobotsManager* pRobotsManager)
{
  // check if we need to connect to StateKeeper. If yes, we need to
  // advertise Robot State and provide rpc methods for StateKeeper.

  if(m_sServerName != "WithoutStateKeeper")
  {
    m_pRobotsManager = pRobotsManager;

    m_Node.advertise("RobotState");

    bool bStatus = RegisterRobotProxyWithStateKeeper();
    if(bStatus == false)
    {
      cout<<"[NetworkManager] Cannot register RobotProxy '"<<m_sLocalSimName<<"' in "<<m_sServerName<<". Please make sure "<<m_sServerName<<" is running!"<<endl;
      return false;
    }
    m_Node.provide_rpc("AddRobotByURDF",&_AddRobotByURDF, this);
    m_Node.provide_rpc("DeleteRobot",&_DeleteRobot, this);
    cout<<"[NetworkManager] Init Robot Network "<<m_sLocalSimName<<" for statekeeper success."<<endl;
  }

  return true;
}


////////////////////////////////////////////////////////////////////////
// Initializes all devices attached to the robot into Node. Notice that
// devices that need to subscribe to the robot (e.g. controllers) are not
// initialized here, but are rather initialized in RobotProxy.
// Notice that if we run in 'with StateKeeper' mode, we need to call this function after
// the main robot has gotten its initial pose from StateKeeper, and been built
// into the ModelGraph.
void NetworkManager::PubRegisterDevicesIfNeeded(SimDeviceManager* pSimDeviceManager)
{
  // 1. check if we need to init device in node.
  if(m_sServerName=="WithoutNetwork")
  {
    cout<<"[LocalSim] Init Robot LocalSim without Network."<<endl;
  }
  else
  {
    m_pSimDeviceManager = pSimDeviceManager;

    // 2. init SamCam
    if(m_pSimDeviceManager->m_SimCamList.size()!=0)
    {
      cout<<"[NetworkManager/PubRegisterDevicesIfNeeded] Try to init "<<m_pSimDeviceManager->m_SimCamList.size()<<" NodeCam."<<endl;

      // provide rpc method for camera to register
      m_Node.provide_rpc("RegsiterCamDevice",&_RegisterCamDevice,this);

      for(unsigned int i = 0; i!= m_pSimDeviceManager->m_vSimDevices.size();i++)
      {
        SimDeviceInfo Device = m_pSimDeviceManager->m_vSimDevices[i];
        string sServiceName = GetFirstName(Device.m_sDeviceName);

        if(m_Node.advertise(sServiceName)==true)
        {
          cout<<"[NetworkManager/PubRegisterDevicesIfNeeded] advertise register "<<sServiceName<<" Success."<<endl;
        }
        else
        {
          cout<<"[NetworkManager/PubRegisterDevicesIfNeeded] advertise register "<<sServiceName<<" Fail."<<endl;
          cout<<"[NetworkManager/PubRegisterDevicesIfNeeded] Cannot init Nextwrok"<<endl;
          exit(-1);
        }
      }
    }

    // 3. init SimGPS
    if(m_pSimDeviceManager->m_SimGPSList.size()!=0)
    {
      map<string, SimGPS*>::iterator iter;
      for(iter = m_pSimDeviceManager->m_SimGPSList.begin();iter!= m_pSimDeviceManager->m_SimGPSList.end();iter++)
      {
        m_Node.advertise(iter->first);
        cout<<"[NetworkManager/CheckIfInitDevices] advertise "<<iter->first<<endl;
      }
    }

    // more device need to be init here

  }
}


////////////////////////////////////////////////////////////////////////
/// REGISTER AND DELETE ROBOTS/DEVICES FROM THE NETWORK
/// Used in StateKeeper and RobotProxy
////////////////////////////////////////////////////////////////////////
// Register Robot Proxy in StateKeeper.
// 1. subscribe to WorldState Topc.
// 2. Send Robot's URDF file to StateKeeper.
// 3. Receive init pose for Robot.
bool NetworkManager::RegisterRobotProxyWithStateKeeper()
{
  // 1. Subscribe to StateKeeper World state topic
  string sServiceName = m_sServerName+"/WorldState";
  if( m_Node.subscribe( sServiceName ) == false )
  {
    cout<<"[NetworkManager/registerRobotProxyWithStateKeeper] Error subscribing to "<<sServiceName<<endl;
    return false;
  }

  // 2. prepare URDF file to StateKeeper
  // 2.1 get Robot URDF (.xml) file
  SimRobot* pSimRobot = m_pRobotsManager->m_mSimRobotsList.begin()->second;
  XMLPrinter printer;
  pSimRobot->GetRobotURDF()->Accept(&printer);

  // 2.2 set request msg
  RegisterRobotProxyReqMsg mRequest;
  string sRobotName = pSimRobot->GetRobotName();
  mRequest.set_proxy_name(m_sLocalSimName);
  mRequest.mutable_urdf()->set_robot_name(sRobotName);
  mRequest.mutable_urdf()->set_xml(printer.CStr());


  // 3 call statekeeper to register robot, (service name, request_msg, reply_msg). Reply message must be empty.
  // init reply msg
  RegisterRobotProxyRepMsg mReply;

  sServiceName = m_sServerName + "/RegisterRobotProxy";
  if( m_Node.call_rpc(sServiceName, mRequest, mReply) == true && mReply.robot_name()==sRobotName)
  {
    Vector6Msg  mInitRobotState = mReply.init_pose();// current robot state

    cout<<"[NetworkManager/registerRobotProxyWithStateKeeper] Set URDF to StateKeeper success"<<endl;

    // 3.1 init time step. this is very important step.
    m_iTimeStep = mReply.time_step();


    // 3.2 init pose state of my robot.
    Eigen::Vector6d ePose;
    ePose<<mInitRobotState.x(), mInitRobotState.y(), mInitRobotState.z(),mInitRobotState.p(), mInitRobotState.q(), mInitRobotState.r();

    /*    m_pRobotsManager->GetMainRobot()->InitPoseOfBodyBaseWRTWorld(ePose);
    m_pRobotsManager->GetMainRobot()->AddRobotInModelGraph();*/

    cout<<"[NetworkManager/registerRobotProxyWithStateKeeper] Robot register success! Get init robot state as x: "<<ePose[0]<<" y: "<<ePose[1]<<" z: "<<ePose[2]<<" p: "<<ePose[3]
       <<" q: "<<ePose[4]<<" r: "<<ePose[5]<<". in Time step: "<<m_iTimeStep<<endl;

    // 3.3 build other robots that already in StateKeeper in our proxy.
    // Read initial pose of them from URDF. This is a trick as their real pose will be set quickly when we
    // sync worldstate message.
    cout<<"[NetworkManager/register] Try to init "<<mReply.urdf_size()<<" previous players."<<endl;
    for(int i=0;i!= mReply.urdf_size();i++)
    {
      // prepare urdf
      const URDFMsg& urdf = mReply.urdf(i);
      string sFullName = urdf.robot_name();
      string sLastName = GetLastName(sFullName);
      XMLDocument doc;
      doc.Parse(urdf.xml().c_str());

      // create previous robot
      URDF_Parser* parse = new URDF_Parser();
      SimRobot* robot = new SimRobot();
      parse->ParseRobot(doc, *robot, sLastName);
      m_pRobotsManager->ImportSimRobot(*robot);

      // TODO: How to add this back into the scene...

      cout<<"[NetworkManager/register] init previous player "<<sFullName<<". Last Name "<<sLastName<<" Success!"<<endl;
    }
    return true;
  }
  else
  {
    cout<<"[register] RobotProxy register unsuccessful"<<endl;
    return false;
  }
}

////////////////////////////////////////////////////////////////////////

/// add or delete robot in Network Manager
void NetworkManager::AddRobotByURDF(RobotProxyAddNewRobotReqMsg& mRequest, RobotProxyAddNewRobotRepMsg& mReply)
{
  // set reply message for StateKeeper
  mReply.set_message("AddNewRobotSuccess");

  // Create New Robot Base on URDF File in my proxy
  XMLDocument doc;
  doc.Parse(mRequest.urdf().xml().c_str());
  string sNewAddRobotName = mRequest.robot_name();
  string sProxyNameOfNewRobot= GetRobotLastName(sNewAddRobotName);

  Eigen::Vector6d ePose;
  ePose<<mRequest.mutable_init_pose()->x(),mRequest.mutable_init_pose()->y(),mRequest.mutable_init_pose()->z(),
      mRequest.mutable_init_pose()->p(),mRequest.mutable_init_pose()->q(),mRequest.mutable_init_pose()->r();

  // add new robot in proxy
  URDF_Parser* parse = new URDF_Parser();
  SimRobot* robot = new SimRobot();
  parse->ParseRobot(doc, *robot, sProxyNameOfNewRobot);
  m_pRobotsManager->ImportSimRobot(*robot);

  // TODO: I don't think this is correct... we may have to change the naming
  // schematic.
  m_pRobotsManager->GetRobot(sNewAddRobotName)->InitPoseOfBodyBaseWRTWorld(ePose);
  //  m_pRobotsManager->GetRobot(sNewAddRobotName)->AddRobotInModelGraph();

  cout<<"[NetworkManager/AddRobotByURDF] Add new robot "<<mRequest.robot_name() <<" success. "<<endl;
}


////////////////////////////////////////////////////////////////////////

void NetworkManager::DeleteRobot(RobotProxyDeleteRobotReqMsg& mRequest, RobotProxyDeleteRobotRepMsg& mReply)
{
  boost::mutex::scoped_lock lock(m_Mutex); // don't let anyone touch the shared resource table...

  // set reply message for StateKeeper
  mReply.set_message("DeleteRobotSuccess");

  // delete robot in our proxy
  string sRobotName = mRequest.robot_name();
  m_pRobotsManager->DeleteRobot(sRobotName);
  cout<<"[NetworkManager/DeleteRobot] Delete Robot "<<sRobotName<<" success."<<endl;
}

////////////////////////////////////////////////////////////////////////

/// Syc WorldState by (1) publish main robot's state and (2) receive world state
// here we read robot state via a vicon device. This state include robot pose state and command.
bool NetworkManager::PublishRobotFullStateToStateKeeper()
{
  boost::mutex::scoped_lock lock(m_Mutex); // don't let anyone touch the shared resource table...

  // 1. set robot name and time step info
  RobotFullStateMsg mRobotFullState;// clear previous robot state information. Very important.
  mRobotFullState.set_robot_name(m_pRobotsManager->GetMainRobot()->GetRobotName());
  mRobotFullState.set_time_step(m_iTimeStep);// mark it as the lastest robot state by time_step +1.

  // 2. set body state info
  vector<string> vAllBodyFullName = m_pRobotsManager->GetMainRobot()->GetAllBodyName();
  for (unsigned int i=0;i!=vAllBodyFullName.size();i++)
  {
    string sBodyName = vAllBodyFullName[i];
    // prepare pose info
    Eigen::Vector3d eOrigin = m_pRobotsManager->m_Scene.m_Phys.GetEntityOrigin(sBodyName);
    Eigen::Matrix3d eBasis = m_pRobotsManager->m_Scene.m_Phys.GetEntityBasis(sBodyName);


    // prepare veloicty info
    Eigen::Vector3d eLinearV = m_pRobotsManager->m_Scene.m_Phys.GetEntityLinearVelocity(sBodyName);
    Eigen::Vector3d eAngularV = m_pRobotsManager->m_Scene.m_Phys.GetEntityAngularVelocity(sBodyName);


    // set pose infor
    BodyStateMsg* mBodyState = mRobotFullState.add_body_state();
    mBodyState->set_body_name(sBodyName);
    mBodyState->mutable_origin()->set_x(eOrigin[0]);
    mBodyState->mutable_origin()->set_y(eOrigin[1]);
    mBodyState->mutable_origin()->set_z(eOrigin[2]);

    mBodyState->mutable_basis()->set_x11(eBasis(0,0));
    mBodyState->mutable_basis()->set_x12(eBasis(0,1));
    mBodyState->mutable_basis()->set_x13(eBasis(0,2));
    mBodyState->mutable_basis()->set_x21(eBasis(1,0));
    mBodyState->mutable_basis()->set_x22(eBasis(1,1));
    mBodyState->mutable_basis()->set_x23(eBasis(1,2));
    mBodyState->mutable_basis()->set_x31(eBasis(2,0));
    mBodyState->mutable_basis()->set_x32(eBasis(2,1));
    mBodyState->mutable_basis()->set_x33(eBasis(2,2));

    // set velocity
    mBodyState->mutable_linear_velocity()->set_x(eLinearV[0]);
    mBodyState->mutable_linear_velocity()->set_y(eLinearV[1]);
    mBodyState->mutable_linear_velocity()->set_z(eLinearV[2]);

    mBodyState->mutable_angular_velocity()->set_x(eAngularV[0]);
    mBodyState->mutable_angular_velocity()->set_y(eAngularV[1]);
    mBodyState->mutable_angular_velocity()->set_z(eAngularV[2]);
  }

  // 4. Publish robot state
  bool bStatus=false;
  while (bStatus==false)
  {
    bStatus=m_Node.publish( "RobotState", mRobotFullState);
    if(bStatus==true)
    {
      if(m_verbocity!=0)
      {
        cout<< "[NetworkManager] Publish " <<m_sLocalSimName<<" State to Statekeeper success. Publish Timestep is "<<m_iTimeStep<<endl;
      }
      return true;
    }
    else
    {
      cout<< "[NetworkManager] ERROR: Publishing RobotState Fail."<<endl;
      return false;
    }
  }
  return true;
}

////////////////////////////////////////////////////////////////////////

bool NetworkManager::ReceiveWorldFullStateFromStateKeeper( )
{
  boost::mutex::scoped_lock lock(m_Mutex); // don't let anyone touch the shared resource table...

  WorldFullStateMsg ws;

  string sServiceName = m_sServerName + "/WorldState";

  // wait until we get the lastest world state
  int iMaxTry=50;
  bool bStatus=false;
  while(bStatus==false)
  {
    if(m_Node.receive(sServiceName, ws )==true && ws.time_step() >= m_pRobotsManager->m_WorldFullState.time_step())
    {
      // update world state in robots manager.
      m_pRobotsManager->UpdateWorldFullState(ws);
      m_pRobotsManager->ApplyWorldFullState();
      m_iTimeStep = ws.time_step();
      bStatus=true;
      if(m_verbocity!=0)
      {
        cout<<"[NetworkManager] Update World state success! size is "<<ws.robot_state_size()<<". Time Step for world state is "<<ws.time_step()<<endl;
      }
    }
    else if(bStatus == false && iMaxTry!=0)
    {
      usleep(50000);
      iMaxTry--;
    }
    else
    {
      cout<<"[NetworkManager/WorldState] Update World state fail!"<<endl;
      return false;
    }
  }
  return true;
}


////////////////////////////////////////////////////////////////////////
/// REGISTER AND DELETE DEVICES FROM THE SIMULATION
/// Code for hal device and RobotProxy.
////////////////////////////////////////////////////////////////////////
void NetworkManager::RegisterCamDevice(RegisterNodeCamReqMsg& mRequest,RegisterNodeCamRepMsg & mReply)
{
  cout<<"[NetworkManager] Node2Cam ask for register in timestep "<<m_iTimeStep<<"."<<endl;

  mReply.set_time_step(m_iTimeStep);
  mReply.set_regsiter_flag(1);
  m_SubscribeNum = m_SubscribeNum +1;
}

// Return 'FALSE' if device is invalid. Otherwise return device name.
string NetworkManager::CheckURI(string sURI)
{
  // Find device in device manager
  string sDeviceName = sURI;

  // check if device is valid

  // return device name if it is valid

  return sDeviceName;
}

void NetworkManager::RegisterCamDeviceByURI(RegisterNodeCamReqMsg& mRequest,RegisterNodeCamRepMsg & mReply)
{
  cout<<"[NetworkManager] Node2Cam ask for register in timestep "<<m_iTimeStep<<"."<<endl;

  string sDeviceName = CheckURI(mRequest.uri());
  if(sDeviceName!="FALSE")
  {
    // Init Device.
    m_pSimDeviceManager->InitDeviceByName(sDeviceName);
    SimCamera* pCam = m_pSimDeviceManager->GetSimCam(sDeviceName);

    mReply.set_time_step(m_iTimeStep);
    mReply.set_regsiter_flag(1);
    mReply.set_channels(pCam->m_nChannels);
    mReply.set_width(pCam->m_nImgWidth);
    mReply.set_height(pCam->m_nImgHeight);

    m_SubscribeNum = m_SubscribeNum +1;
    cout<<"HAL reuqest for use "<<sDeviceName<<". Device Ready!"<<endl;
  }
  else
  {
    mReply.set_time_step(m_iTimeStep);
    mReply.set_regsiter_flag(0);
    cout<<"HAL reuqest for use "<<sDeviceName<<". Device Invalid!"<<endl;
  }
}

////////////////////////////////////////////////////////////////////////

void NetworkManager::RegisterControllerDevice(RegisterControllerReqMsg& mRequest,RegisterControllerRepMsg & mReply)
{
  cout<<"[NetworkManager] Node2Cam ask for register in timestep "<<m_iTimeStep<<"."<<endl;
  mReply.set_time_step(m_iTimeStep);
  m_SubscribeNum = m_SubscribeNum +1;

  // robot proxy subscribe to this controller device for command
  string sServiceName = mRequest.controller_name()+"-for-"+m_sLocalSimName; // service name, e.g. mController-for-Proxy1.
  if( m_Node.subscribe(sServiceName)==false )
  {
    cout<<"[NetworkManager/RegisterControllerDevice] Fatal error! Cannot subscribe to "<<sServiceName<<". Please make sure service is running."<<endl;
  }
}

////////////////////////////////////////////////////////////////////////
/// UPDATE AND PUBLISH INFO
////////////////////////////////////////////////////////////////////////

bool NetworkManager::UpdateNetWork()
{
  if(m_sServerName == "WithoutNetwork")
  {
    return true;
  }

  // there must be at least one hal device subscribe to proxy
  //        if(m_SubscribeNum!=0)
  //        {
  if(m_verbocity!=0)
  {
    cout<<"*****************************************************************************"<<endl;
  }

  m_iTimeStep++;

  // 2. Public Robot State Info (to statekeeper)
  if(m_sServerName!="WithoutStateKeeper")
  {
    if(PublishRobotFullStateToStateKeeper() == false)
    {
      cout<<"[NetworkManager] Cannot Publish Robot State To StateKeeper!!! You May Disconnected From Server!! "<<endl;
      return false;
    }

    if(ReceiveWorldFullStateFromStateKeeper()== false)
    {
      cout<<"[NetworkManager] Cannot Receive World State from StateKeeper!!! You May Disconnected From Server!! "<<endl;
      return false;
    }
  }

  // 1. Publish and receive all Device Info (to robot)
  for(unsigned int i=0;i!= m_pSimDeviceManager->m_vSimDevices.size();i++)
  {
    SimDeviceInfo Device = m_pSimDeviceManager->m_vSimDevices[i];
    string sDeviceType = Device.m_sDeviceType;
    string sDeviceName = Device.m_sDeviceName;
    bool bDeviceOn = Device.m_bDeviceOn;

    if( sDeviceType == "Camera" && bDeviceOn == true)                      // update Camera info
    {
      if( PublishSimCamBySensor(sDeviceName) == false)
      {
        return false;
      }
    }
    else if(sDeviceType == "GPS" && bDeviceOn == true)                     // update GPS info
    {
      if(PublishGPS(sDeviceName) == false )
      {
        return false;
      }
    }
    else if(sDeviceType == "Controller" && bDeviceOn == true)
    {
      if(ReceiveControlInfo(sDeviceName) == false)  // update controller info
      {
        return false;
      }
    }
  }
  //        }
  return true;

}

////////////////////////////////////////////////////////////////////////

// To publish device information
// Receive information from all controller.
// (we may have more than one controller here.)

bool NetworkManager::ReceiveControlInfo(string sDeviceName)
{
  // get controller
  SimpleController* pController = m_pSimDeviceManager->GetSimpleController(sDeviceName);

  string sServiceName=sDeviceName+"/Controller";

  // wait until we get the lastest world state

  int                iUpdateFlag=0;
  CommandMsg         Command;// current command from robot
  CommandMsg         TryCommand;

  while(1)
  {
    if(m_Node.receive( sServiceName, TryCommand )==true)
    {
      Command=TryCommand;
      iUpdateFlag=1;
      cout<<"[Command] Update Command success! Command time step is "<<Command.time_step()<<endl;
    }
    else if(m_Node.receive( sServiceName, TryCommand )==false && iUpdateFlag==1 )
    {
      cout<<"[Command] Update Command success! Command time step is "<<Command.time_step()<<endl;

      Eigen::Vector6d command;
      command<<Command.x(), Command.y(), Command.z(), Command.p(),Command.q(),Command.r();

      vector<string> vBodyFullName;
      for(int j =0; j!= Command.body_name_size();j++)
      {
        string sBodyFullName = Command.body_name(j);
        vBodyFullName.push_back(sBodyFullName);
      }

      pController->UpdateCommand(vBodyFullName, command);

      return true;
    }
    else if(m_Node.receive( sServiceName, TryCommand )==false && iUpdateFlag==0 )
    {
      //                 cout<<"[Command] Update command fail..Break."<<endl;
      return false;
    }
  }
}

///////////////////////////////////////////////////////////////////////

bool NetworkManager::ReceiveSimpleControllerInfo()
{
  return true;
}

////////////////////////////////////////////////////////////////////////

bool NetworkManager::ReceiveCarControllerInfo()
{
  return true;
}

////////////////////////////////////////////////////////////////////////

// publish SimCamera by sensor. e.g. one SimCamera may have RGB and Depth sensor that provide us images.
// here we will publish image from all these sensor.
bool NetworkManager::PublishSimCamBySensor(string sCamName)
{
  NodeCamMsg mNodeCamMsg;
  mNodeCamMsg.set_time_step(m_iTimeStep);
  int image_size = 0;

  // save image to NodeCamMsg data struct
  for(unsigned int i = 0; i!= m_pSimDeviceManager->m_vSimDevices.size(); i++)
  {
    SimDeviceInfo Device = m_pSimDeviceManager->m_vSimDevices[i];
    string sDeviceName = Device.m_sDeviceName;

    if(sDeviceName == sCamName)
    {
      for (unsigned int j=0;j!=Device.m_vSensorList.size();j++)
      {
        image_size = image_size + 1;

        // get one camera sensor
        string sSensorName = Device.m_vSensorList[j];
        SimCamera* pSimCam= m_pSimDeviceManager->GetSimCam(sSensorName);

        // set NodeCamMsg info
        NodeCamImageMsg *pImage = mNodeCamMsg.add_image();

        // ------------------------------------------------ for gray scale image
        if(pSimCam->m_iCamType == 1)
        {
          char* pImgbuf= (char*)malloc (pSimCam->m_nImgWidth * pSimCam->m_nImgHeight);
          if(pSimCam->capture(pImgbuf)==true)
          {
            pImage->set_image(pImgbuf, pSimCam->m_nImgWidth * pSimCam->m_nImgHeight);
          }
          else
          {
            return false;
          }
          free(pImgbuf);
        }
        // ------------------------------------------------------- for RGB image
        else if(pSimCam->m_iCamType == 2)
        {
          char* pImgbuf= (char*)malloc (pSimCam->m_nImgWidth * pSimCam->m_nImgHeight *3);
          if(pSimCam->capture(pImgbuf)==true)
          {
            pImage->set_image(pImgbuf,pSimCam->m_nImgWidth * pSimCam->m_nImgHeight*3);
          }
          else
          {
            return false;
          }
          free(pImgbuf);
        }
        // ----------------------------------------------------- for depth image
        else if(pSimCam->m_iCamType == 5)
        {
          float* pImgbuf = (float*) malloc( pSimCam->m_nImgWidth * pSimCam->m_nImgHeight * sizeof(float) );
          if(pSimCam->capture(pImgbuf)==true)
          {
            pImage->set_image(pImgbuf, pSimCam->m_nImgWidth * pSimCam->m_nImgHeight * sizeof(float));
          }
          else
          {
            return false;
          }
          free(pImgbuf);
        }

        pImage->set_image_type(pSimCam->m_iCamType);
        pImage->set_image_height(pSimCam->m_nImgHeight);
        pImage->set_image_width(pSimCam->m_nImgWidth);
      }
    }
  }

  mNodeCamMsg.set_size(image_size);

  // publish
  sCamName = GetFirstName(sCamName);

  bool bStatus=m_Node.publish(sCamName,mNodeCamMsg);

  if( bStatus==false)
  {
    cout<<"["<<m_sLocalSimName<<"/NodeCam] ERROR: publishing images fail.."<<endl;
    return false;
  }

  if(m_verbocity!=0)
  {
    cout<<"["<<m_sLocalSimName<<"/NodeCam] Publsih NodeCam image success." <<endl;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////

// publish GPS by device name

bool NetworkManager::PublishGPS(string sDeviceName)
{
  Eigen::Vector3d pose;
  m_pSimDeviceManager->GetSimGPS(sDeviceName)->GetPose(pose);

  GPSMsg mGPSMSg;
  mGPSMSg.set_time_step(m_iTimeStep);
  mGPSMSg.set_x(pose[0]);
  mGPSMSg.set_y(pose[1]);
  mGPSMSg.set_y(pose[2]);

  sDeviceName = GetFirstName(sDeviceName);

  if(m_verbocity!=0)
  {
    cout<<"[NodeGPS] Try to publish "<<sDeviceName<< ". x="<<pose[0]<<" y="<<pose[1]<<" z="<<pose[2]<< ". Time step is "<<mGPSMSg.time_step()<<"."<<endl;
  }

  bool bStatus=false;
  while(bStatus==false)
  {
    bStatus=m_Node.publish(sDeviceName,mGPSMSg);
    if( bStatus==false)
    {
      printf("[NodeGPS] ERROR: publishing GPS fail. Try again.\n" );
    }
  }
  if(m_verbocity!=0)
  {
    cout<<"[NodeGPS] publsih GPS success." <<endl;
  }

  return true;
}
