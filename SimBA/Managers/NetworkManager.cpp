#include <Managers/NetworkManager.h>


/************************************************************
  *
  * INITIALIZER
  *
  ***********************************************************/

bool NetworkManager::Init(string sProxyName, string sServerName, int verbocity){
  // Check if we need to init Node for LocalSim
  m_sServerName = sServerName;
  if(m_sServerName == "WithoutNetwork"){
    cout<<"[NetworkManager] Skip Network due to WithoutNetwork Mode"<<endl;
    return true;
  }

  // Else, Init Node connection in LocalSim and provide
  // relative RPC method.
  m_sLocalSimName  = sProxyName;
  m_verbocity      = verbocity;
  m_iNodeClients   = 0;

  m_Node.set_verbocity(m_verbocity ); // make some noise on errors
  bool worked = m_Node.init(m_sLocalSimName);
  if(!worked){
    cout<<"[NetworkManager] Init Node '"<<
          m_sLocalSimName<<"' failed..."<<endl;
    return false;
  }
  return true;
}

/************************************************************
  *
  * NODE FUNCTIONS
  *
  ***********************************************************/

// Return 'FALSE' if device is invalid. Otherwise return device name.

string NetworkManager::CheckURI(string sURI){
  // Find device in device manager
  string sDeviceName = sURI;
  // TODO: check if device is valid

  // return device name if it is valid
  return sDeviceName;
}


////////////////////////////////////////////////////////////////////////
// Initializes all devices attached to the robot into Node.
// Notice that if we run in 'WithStateKeeper' mode, we need to call this
// function after the main robot has gotten its initial pose from StateKeeper,
// and been built into the ModelGraph.

void NetworkManager::RegisterDevices(SimDevices* pSimDevices){

  // Check if we need to init device in node.
  if(m_sServerName=="WithoutNetwork"){
    cout<<"[NetworkManager/RegisterDevices]"<<
          "Skip! Init Robot LocalSim without Network."<<endl;
  }

  else{
    m_pSimDevices = pSimDevices;

    /*******************
    // SimSensors
    ********************/

    ///////////
    // SimCam
    ///////////
    if(m_pSimDevices->m_SimCamList.size()!=0){
      cout<<"[NetworkManager/RegisterDevices] Trying to init "<<
            m_pSimDevices->m_SimCamList.size()<<" NodeCams."<<endl;

      // provide rpc method for camera to register
      m_Node.provide_rpc("RegsiterCamDevice",&_RegisterCamDevice,this);

      for(unsigned int i = 0;
          i!= m_pSimDevices->m_vSimDevices.size();
          i++){
        SimDeviceInfo Device = m_pSimDevices->m_vSimDevices[i];
        string sServiceName = GetFirstName(Device.m_sDeviceName);
        if(m_Node.advertise(sServiceName)==true){
          cout<<"[NetworkManager/RegisterDevices]"<<
                " Advertising "<<sServiceName<<" --> Success."<<endl;
        }
        else{
          cout<<"[NetworkManager/RegisterDevices]"<<
                " Advertising "<<sServiceName<<" --> Fail."<<endl;
          exit(-1);
        }
      }
    }

    ///////////////
    // SimGPS
    ///////////////
    if(m_pSimDevices->m_SimGPSList.size()!=0){
      map<string, SimGPS*>::iterator iter;
      for(iter = m_pSimDevices->m_SimGPSList.begin();
          iter!= m_pSimDevices->m_SimGPSList.end();
          iter++){
        m_Node.advertise(iter->first);
        cout<<"[NetworkManager/CheckIfInitDevices] advertise "<<
              iter->first<<endl;
      }
    }

    /*******************
    // SimControllers
    ********************/

  }
}

////////////////////////////////////////////////////////////////////////
/// REGISTER AND DELETE DEVICES FROM THE SIMULATION

void NetworkManager::RegisterCamDevice(RegisterNodeCamReqMsg& mRequest,
                                       RegisterNodeCamRepMsg & mReply){
  cout<<"[NetworkManager/RegisterCamDevice]"<<
        " Node2Cam ask for register in timestep "<<m_iTimeStep<<"."<<endl;

  string sDeviceName = CheckURI(mRequest.uri());
  if(sDeviceName!="FALSE"){
    SimCamera* pCam = m_pSimDevices->GetSimCam(sDeviceName);
    mReply.set_time_step(m_iTimeStep);
    mReply.set_regsiter_flag(1);
    mReply.set_channels(pCam->m_nChannels);
    mReply.set_width(pCam->m_nImgWidth);
    mReply.set_height(pCam->m_nImgHeight);
    m_iNodeClients = m_iNodeClients + 1;
    cout<<"[NetworkManager/RegisterCamDevice] HAL requesting "<<
          sDeviceName<<". Device Ready!"<<endl;
  }
  else{
    mReply.set_time_step(m_iTimeStep);
    mReply.set_regsiter_flag(0);
    cout<<"[NetworkManager/RegisterCamDevice] HAL requesting "<<
          sDeviceName<<". Device Invalid!"<<endl;
  }
}

////////////////////////////////////////////////////////////////////////

void NetworkManager::RegisterControllerDevice(
    RegisterControllerReqMsg& mRequest,RegisterControllerRepMsg & mReply){
  mReply.set_success(true);

  // Check to see if you can even subscribe to this topic.
  if( m_Node.subscribe(mRequest.topic())==false ){
    cout<<"[NetworkManager/RegisterControllerDevice] Fatal error! "
          "Cannot subscribe to "<<mRequest.topic()
       <<". Please make sure service is running."<<endl;
  }
  else{
    m_iNodeClients = m_iNodeClients + 1;
  }
}

////////////////////////////////////////////////////////////////////////
/// UPDATE AND PUBLISH INFO
////////////////////////////////////////////////////////////////////////

bool NetworkManager::UpdateNetwork(){
  m_iTimeStep++;
  if(m_sServerName == "WithoutNetwork"){
    return true;
  }
  if(m_verbocity != 0){
    /// What is this supposed to do...?
    cout<<"*************************************************************"<<endl;
  }
  if(m_sServerName == "WithStateKeeper"){
    if(PublishRobotToStateKeeper() == false){
      cout<<"[NetworkManager] Cannot Publish Robot State To StateKeeper!!!"<<
            " You might be disconnected from the server..."<<endl;
      return false;
    }
    if(ReceiveWorldFromStateKeeper() == false){
      cout<<"[NetworkManager] Cannot Receive World State from StateKeeper!!!"<<
            " You may be disconnected from the server... "<<endl;
      return false;
    }
  }

  for(unsigned int i=0;i!= m_pSimDevices->m_vSimDevices.size();i++){
    SimDeviceInfo Device = m_pSimDevices->m_vSimDevices[i];
    string sDeviceType = Device.m_sDeviceType;
    string sDeviceName = Device.m_sDeviceName;
    bool bDeviceOn = Device.m_bDeviceOn;
    // Update Camera
    if( sDeviceType == "Camera" && bDeviceOn == true){
      if( PublishSimCamBySensor(sDeviceName) == false){
        return false;
      }
    }
    // Update GPS
    else if(sDeviceType == "GPS" && bDeviceOn == true){
      if(PublishGPS(sDeviceName) == false ){
        return false;
      }
    }
    // update Controller Info
    else if(sDeviceType == "Controller" && bDeviceOn == true){
      if(ReceiveControllerInfo(sDeviceName) == false){
        return false;
      }
    }
  }
  return true;
}

////////////////////////////////////////////////////////////////////////

// Publish SimCamera by Sensor.
bool NetworkManager::PublishSimCamBySensor(string sCamName){
  NodeCamMsg mNodeCamMsg;
  mNodeCamMsg.set_time_step(m_iTimeStep);
  int image_size = 0;

  // save image to NodeCamMsg data struct
  for(unsigned int i = 0; i!= m_pSimDevices->m_vSimDevices.size(); i++){
    SimDeviceInfo Device = m_pSimDevices->m_vSimDevices[i];
    string sDeviceName = Device.m_sDeviceName;

    if(sDeviceName == sCamName){
      for (unsigned int j=0;j!=Device.m_vSensorList.size();j++){
        image_size = image_size + 1;

        // get one camera sensor
        string sSensorName = Device.m_vSensorList[j];
        SimCamera* pSimCam= m_pSimDevices->GetSimCam(sSensorName);

        // set NodeCamMsg info
        NodeCamImageMsg *pImage = mNodeCamMsg.add_image();

        ////////////
        // A grayscale image
        ////////////
        if(pSimCam->m_iCamType == 1){
          char* pImgbuf = (char*)malloc (pSimCam->m_nImgWidth *
                                         pSimCam->m_nImgHeight);
          if(pSimCam->capture(pImgbuf)==true){
            pImage->set_image(pImgbuf, pSimCam->m_nImgWidth *
                              pSimCam->m_nImgHeight);
          }
          else{
            return false;
          }
          free(pImgbuf);
        }
        ////////////
        // An RGB image
        ////////////
        else if(pSimCam->m_iCamType == 2){
          char* pImgbuf= (char*)malloc (pSimCam->m_nImgWidth *
                                        pSimCam->m_nImgHeight *3);
          if(pSimCam->capture(pImgbuf)==true){
            pImage->set_image(pImgbuf,pSimCam->m_nImgWidth *
                              pSimCam->m_nImgHeight*3);
          }
          else{
            return false;
          }
          free(pImgbuf);
        }
        ////////////
        // A depth image
        ////////////
        else if(pSimCam->m_iCamType == 5){
          float* pImgbuf = (float*) malloc( pSimCam->m_nImgWidth *
                                            pSimCam->m_nImgHeight *
                                            sizeof(float) );
          if(pSimCam->capture(pImgbuf)==true){
            pImage->set_image(pImgbuf, pSimCam->m_nImgWidth *
                              pSimCam->m_nImgHeight *
                              sizeof(float));
          }
          else{
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

  // Publish the info
  sCamName = GetFirstName(sCamName);
  bool bStatus=m_Node.publish(sCamName,mNodeCamMsg);
  if( bStatus==false){
    cout<<"["<<m_sLocalSimName<<
          "/NodeCam] ERROR: publishing images fail.."<<endl;
    return false;
  }
  if(m_verbocity!=0){
    cout<<"["<<m_sLocalSimName<<
          "/NodeCam] Publsih NodeCam image success." <<endl;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////
// publish GPS by device name

bool NetworkManager::PublishGPS(string sDeviceName){
  Eigen::Vector3d pose;
  m_pSimDevices->GetSimGPS(sDeviceName)->GetPose(pose);

  GPSMsg mGPSMSg;
  mGPSMSg.set_time_step(m_iTimeStep);
  mGPSMSg.set_x(pose[0]);
  mGPSMSg.set_y(pose[1]);
  mGPSMSg.set_y(pose[2]);

  sDeviceName = GetFirstName(sDeviceName);

  if(m_verbocity!=0){
    cout<<"[NodeGPS] Try to publish "<<sDeviceName<< ". x="<<pose[0]<<
          " y="<<pose[1]<<" z="<<pose[2]<<
          ". Time step is "<<mGPSMSg.time_step()<<"."<<endl;
  }

  bool bStatus=false;
  while(bStatus==false){
    bStatus=m_Node.publish(sDeviceName,mGPSMSg);
    if( bStatus==false){
      printf("[NodeGPS] ERROR: publishing GPS fail. Try again.\n" );
    }
  }
  if(m_verbocity!=0){
    cout<<"[NodeGPS] publsih GPS success." <<endl;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////
// Receive information from all controllers hooked up to HAL.
// (we may have more than one controller here.)

bool NetworkManager::ReceiveControllerInfo(string sDeviceName){
  Controller* pController = m_pSimDevices->GetController(sDeviceName);
  string sServiceName = sDeviceName+"/Controller";

  // TODO: Synch vs. asynch
  // SYNCHRONIZED PROTOCOL:

  // Car Controller
  if(static_cast<CarController*>( pController ) != NULL){
    CarController* pCarController = (CarController*) pController;
    VehicleMsg Command;
    if(m_Node.receive( sServiceName, Command)==true){
      pCarController->UpdateCommand(Command);
      return true;
    }
    else{
      cout<<"Couldn't get the command for the Car Controller"<<endl;
      return false;
    }
  }

  // Simple Controller (aka Camera Body control?)
  else if(static_cast<SimpleController*>( pController ) != NULL){
    SimpleController* pSimpController = (SimpleController*) pController;
    PoseMsg Command;
    if(m_Node.receive( sServiceName, Command)==true){
      pSimpController->UpdateCommand(Command);
      return true;
    }
    else{
      cout<<"Couldn't get the command for the Simple Controller"<<endl;
      return false;
    }
  }
}


/************************************************************
  *
  * STATEKEEPER FUNCTIONS
  *
  ***********************************************************/

// Used to commmunicate with the StateKeeper, if it's initialized.
bool NetworkManager::RegisterRobot(RobotsManager* pRobotsManager){
  // Check if we need to connect to StateKeeper.
  if(m_sServerName == "WithoutNetwork"){
    cout<<"[NetworkManager/RegisterRobot]"<<
          " Skip due to WithoutNetwork mode"<<endl;
  }

  else if(m_sServerName == "WithoutStateKeeper"){
    cout<<"[NetworkManager/RegisterRobot]"<<
          " Skip due to WithoutStateKeeper mode"<<endl;
  }

  // We have a StateKeeper! Go publish. Now.
  else if(m_sServerName == "WithStateKeeper"){
    m_pRobotsManager = pRobotsManager;
    m_Node.advertise("RobotState");
    bool bStatus = RegisterWithStateKeeper();
    if(bStatus == false){
      cout<<"[NetworkManager] Cannot register LocalSim '"<<
            m_sLocalSimName<<"' in "<<m_sServerName<<
            ". Please make sure "<<m_sServerName<<" is running!"<<endl;
      return false;
    }
    // TODO: Allow these methods; not sure if they work yet, though.
    m_Node.provide_rpc("AddRobotByURDF",&_AddRobotByURDF, this);
    m_Node.provide_rpc("DeleteRobot",&_DeleteRobot, this);
    cout<<"[NetworkManager] Init Robot Network "<<m_sLocalSimName<<
          " for statekeeper success."<<endl;
  }
  return true;
}


////////////////////////////////////////////////////////////////////////
/// REGISTER AND DELETE ROBOTS/DEVICES FROM STATEKEEPER
/// 1. subscribe to WorldState Topic.
/// 2. Send Robot's URDF file to StateKeeper.
/// 3. Receive init pose for Robot.

bool NetworkManager::RegisterWithStateKeeper()
{
  // 1. Subscribe to StateKeeper World state topic
  string sServiceName = m_sServerName+"/WorldState";
  if( m_Node.subscribe(sServiceName) == false ){
    cout<<"[NetworkManager/RegisterWithStateKeeper]"<<
          " Error subscribing to "<<sServiceName<<endl;
    return false;
  }

  // 2. prepare URDF file to StateKeeper
  //  - Get Robot URDF (.xml) file
  //  - Set request msg
  SimRobot* pSimRobot = m_pRobotsManager->m_mSimRobotsList.begin()->second;
  XMLPrinter printer;
  pSimRobot->GetRobotURDF()->Accept(&printer);
  RegisterLocalSimReqMsg mRequest;
  string sRobotName = pSimRobot->GetRobotName();
  mRequest.set_proxy_name(m_sLocalSimName);
  mRequest.mutable_urdf()->set_robot_name(sRobotName);
  mRequest.mutable_urdf()->set_xml(printer.CStr());

  // 3. Call StateKeeper to register robot: service name, request_msg,
  // reply_msg. Reply message must be empty.
  RegisterLocalSimRepMsg mReply;
  sServiceName = m_sServerName + "/RegisterLocalSim";
  if(m_Node.call_rpc(sServiceName, mRequest, mReply) == true &&
      mReply.robot_name()==sRobotName){
    Vector6Msg  mInitRobotState = mReply.init_pose();
    cout<<"[NetworkManager/RegisterWithStateKeeper]"<<
          " Set URDF to StateKeeper success"<<endl;

    // 3.1 init time step. this is very important step.
    m_iTimeStep = mReply.time_step();


    // 3.2 init pose state of my robot.
    Eigen::Vector6d ePose;
    ePose<<mInitRobotState.x(), mInitRobotState.y(), mInitRobotState.z(),
        mInitRobotState.p(), mInitRobotState.q(), mInitRobotState.r();

    cout<<"[NetworkManager/RegisterWithStateKeeper] Robot register success!"<<
          " Get init robot state as x: "<<ePose[0]<<" y: "<<ePose[1]<<
          " z: "<<ePose[2]<<" p: "<<ePose[3]<<" q: "<<ePose[4]<<
          " r: "<<ePose[5]<<". in Time step: "<<m_iTimeStep<<endl;

    // 3.3 build other robots that already in StateKeeper in our proxy.
    // Read initial pose of them from URDF. This is a trick as their real
    // pose will be set quickly when we sync worldstate message.
    cout<<"[NetworkManager/register] Try to init "<<mReply.urdf_size()<<
          " previous players."<<endl;
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

      cout<<"[NetworkManager/register] init previous player "<<sFullName<<
            ". Last Name "<<sLastName<<" Success!"<<endl;
    }
    return true;
  }
  else
  {
    cout<<"[register] LocalSim register unsuccessful"<<endl;
    return false;
  }
}

////////////////////////////////////////////////////////////////////////
/// Add a new robot to the Sim (through StateKeeper)

void NetworkManager::AddRobotByURDF(LocalSimAddNewRobotReqMsg& mRequest,
                                    LocalSimAddNewRobotRepMsg& mReply)
{
  // set reply message for StateKeeper
  mReply.set_message("AddNewRobotSuccess");

  // Create New Robot Base on URDF File in my proxy
  XMLDocument doc;
  doc.Parse(mRequest.urdf().xml().c_str());
  string sNewAddRobotName = mRequest.robot_name();
  string sProxyNameOfNewRobot= GetRobotLastName(sNewAddRobotName);

  Eigen::Vector6d ePose;
  ePose<<mRequest.mutable_init_pose()->x(), mRequest.mutable_init_pose()->y(),
      mRequest.mutable_init_pose()->z(), mRequest.mutable_init_pose()->p(),
      mRequest.mutable_init_pose()->q(), mRequest.mutable_init_pose()->r();

  // add new robot in proxy
  URDF_Parser* parse = new URDF_Parser();
  SimRobot* robot = new SimRobot();
  parse->ParseRobot(doc, *robot, sProxyNameOfNewRobot);
  m_pRobotsManager->ImportSimRobot(*robot);

  cout<<"[NetworkManager/AddRobotByURDF] Add new robot "<<
        mRequest.robot_name() <<" success. "<<endl;
}

////////////////////////////////////////////////////////////////////////
/// Delete an existing robot from the Sim (through StateKeeper)

void NetworkManager::DeleteRobot(LocalSimDeleteRobotReqMsg& mRequest,
                                 LocalSimDeleteRobotRepMsg& mReply){
  // don't let anyone touch the shared resource table...
  boost::mutex::scoped_lock lock(m_Mutex);

  // set reply message for StateKeeper
  mReply.set_message("DeleteRobotSuccess");

  // delete robot in our proxy
  string sRobotName = mRequest.robot_name();
  m_pRobotsManager->DeleteRobot(sRobotName);
  cout<<"[NetworkManager/DeleteRobot] Delete Robot "<<
        sRobotName<<" success."<<endl;
}

////////////////////////////////////////////////////////////////////////
/// Sync WorldState by
/// (1) Publishing main robot's state and
/// (2) Receiving the world state
/// We read the robot state via a vicon device. This state includes the robot
/// pose, state, and current command.
bool NetworkManager::PublishRobotToStateKeeper(){
  // don't let anyone touch the shared resource table...
  boost::mutex::scoped_lock lock(m_Mutex);

  // 1. Set robot name and time step info
  RobotFullStateMsg mRobotFullState;
  mRobotFullState.set_robot_name(
        m_pRobotsManager->GetMainRobot()->GetRobotName());
  // mark it as the lastest robot state by time_step +1.
  mRobotFullState.set_time_step(m_iTimeStep);

  // 2. Set body state info
  vector<string> vAllBodyFullName =
      m_pRobotsManager->GetMainRobot()->GetAllBodyName();
  for (unsigned int i=0;i!=vAllBodyFullName.size();i++)
  {
    string sBodyName = vAllBodyFullName[i];
    // prepare pose info
    Eigen::Vector3d eOrigin =
        m_pRobotsManager->m_Scene.m_Phys.GetEntityOrigin(sBodyName);
    Eigen::Matrix3d eBasis =
        m_pRobotsManager->m_Scene.m_Phys.GetEntityBasis(sBodyName);

    // prepare veloicty info
    Eigen::Vector3d eLinearV =
        m_pRobotsManager->m_Scene.m_Phys.GetEntityLinearVelocity(sBodyName);
    Eigen::Vector3d eAngularV =
        m_pRobotsManager->m_Scene.m_Phys.GetEntityAngularVelocity(sBodyName);

    // set pose info
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
        cout<<"[NetworkManager] Publish " <<m_sLocalSimName<<
              " State to Statekeeper success. Publish Timestep is "<<
              m_iTimeStep<<endl;
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

bool NetworkManager::ReceiveWorldFromStateKeeper(){
  boost::mutex::scoped_lock lock(m_Mutex);

  WorldFullStateMsg ws;

  string sServiceName = m_sServerName + "/WorldState";

  // wait until we get the lastest world state
  int iMaxTry=50;
  bool bStatus=false;
  while(bStatus==false){
    if(m_Node.receive(sServiceName, ws )==true && ws.time_step() >=
       m_pRobotsManager->m_WorldFullState.time_step()){
      // update world state in robots manager.
      m_pRobotsManager->UpdateWorldFullState(ws);
      m_pRobotsManager->ApplyWorldFullState();
      m_iTimeStep = ws.time_step();
      bStatus=true;
      if(m_verbocity!=0){
        cout<<"[NetworkManager] Update World state success! Size is "<<
              ws.robot_state_size()<<". Time Step for world state is "<<
              ws.time_step()<<endl;
      }
    }
    else if(bStatus == false && iMaxTry!=0){
      usleep(50000);
      iMaxTry--;
    }
    else{
      cout<<"[NetworkManager/WorldState] Update World state fail!"<<endl;
      return false;
    }
  }
  return true;
}
