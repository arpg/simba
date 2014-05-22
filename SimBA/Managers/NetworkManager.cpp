#include <Managers/NetworkManager.h>

/************************************************************
  *
  * INITIALIZER
  *
  ***********************************************************/

bool NetworkManager::Init(string sProxyName, string sServerName, int verbosity){
  m_sServerName = sServerName;
  debug_level_ = verbosity;
  if(m_sServerName == "WithoutNetwork"){
    LOG(debug_level_) << "Skip Network due to WithoutNetwork Mode";
    return true;
  }
  // If we do need it, Init Node connection in LocalSim and provide
  // relative RPC method.
  m_sLocalSimName  = sProxyName;
  m_iNodeClients   = 0;
  node_.set_verbosity(1);
  bool worked = node_.init(m_sLocalSimName);
  if(!worked){
    LOG(debug_level_) << "FAILURE: Initialized node for " << m_sLocalSimName;
    return false;
  }
  LOG(debug_level_) << "SUCCESS: Initialized node for " << m_sLocalSimName;
  return true;
}

/************************************************************
  *
  * URI PARSERS
  * These are incredibly useful; they provide parameters,
  * and they're the only things that tell us what
  * devices the user wants on or off.
  *
  ***********************************************************/

std::map<string, string> NetworkManager::ParseURI(string sURI){
  std::map<string, string> uri_contents;
  std::size_t found = sURI.find(":");
  if(found!=std::string::npos){
    uri_contents.insert(std::pair<string,string>("device",
                                                 sURI.substr(0, found)));
    found = found+2;
    sURI = sURI.substr(found);
  }
  while(sURI.find(",")!=std::string::npos){
    std::size_t found_name = sURI.find("=");
    std::size_t found_value = sURI.find(',');
    uri_contents.insert(std::pair<string,string>(
        sURI.substr(0, (found_name)),
        sURI.substr(found_name+1,
                    (found_value-found_name-1))));
    sURI = sURI.substr(found_value+1);
  }
  std::size_t found_name = sURI.find("=");
  std::size_t found_value = sURI.find(']');
  uri_contents.insert(std::pair<string,string>(
      sURI.substr(0, (found_name)),
      sURI.substr(found_name+1,
                  (found_value-found_name-1))));
  LOG(debug_level_) << "URI parameters:";
  for(map<string, string>::iterator it = uri_contents.begin();
      it != uri_contents.end();
      it++){
    LOG(debug_level_) << it->first << " is set to " << it->second;
  }
  return uri_contents;
}

// Return 'FALSE' if device is invalid. Otherwise return device name.

string NetworkManager::CheckURI(string sURI){
  // Find device in device manager
  std::map<string, string> uri_contents = ParseURI(sURI);
  string sDeviceName = uri_contents["name"]+"@"+uri_contents["sim"];
  vector<SimDeviceInfo*> pDevices =
      m_pSimDevices->GetAllRelatedDevices(sDeviceName);
  if(!pDevices.empty()){
    // Check for different devices here. Not sure how else to do this...
    if(uri_contents["device"]=="openni"){
      if(uri_contents["rgb"]=="1"){
        m_pSimDevices->m_vSimDevices["RGB_"+sDeviceName]->m_bDeviceOn = true;
      }
      if(uri_contents["depth"]=="1"){
        m_pSimDevices->m_vSimDevices["Depth_"+sDeviceName]->m_bDeviceOn = true;
      }
    }
    /// NodeCar controller
    if(uri_contents["device"]=="NodeCar"){
        m_pSimDevices->m_vSimDevices[sDeviceName]->m_bDeviceOn = true;
        LOG(debug_level_) << "Got a NodeCar Controller!";
      }
    return sDeviceName;
  }
  return "FALSE";
}

/************************************************************
  *
  * NODE FUNCTIONS
  *
  ***********************************************************/

////////////////////////////////////////////////////////////////////////
// Initializes all devices attached to the robot into Node.
// Notice that if we run in 'WithStateKeeper' mode, we need to call this
// function after the main robot has gotten its initial pose from StateKeeper,
// and been built into the ModelGraph.

void NetworkManager::RegisterDevices(SimDevices* pSimDevices){
  m_pSimDevices = pSimDevices;

  // Check if we need to init devices in Node.
  if (m_sServerName=="WithoutNetwork") {
    LOG(debug_level_) << "Skip! Init LocalSim without Network.";
    // Turn all of our devices on, since it doesn't matter.
    for(map<string, SimDeviceInfo*>::iterator it =
        m_pSimDevices->m_vSimDevices.begin();
        it != m_pSimDevices->m_vSimDevices.end();
        it++){
      SimDeviceInfo* Device = it->second;
      Device->m_bDeviceOn = true;
    }
  } else {
    for(map<string, SimDeviceInfo*>::iterator it =
        m_pSimDevices->m_vSimDevices.begin();
        it != m_pSimDevices->m_vSimDevices.end();
        it++){
      SimDeviceInfo* Device = it->second;

      /*******************
      * SimSensors
      ********************/

      /// CAMERAS
      if (Device->m_sDeviceType=="Camera" && !Device->m_bHasAdvertised) {
        vector<SimDeviceInfo*> related_devices =
            m_pSimDevices->GetAllRelatedDevices(Device->GetBodyName());
        SimCamera* pCam = (SimCamera*) related_devices.at(0);
        // provide rpc method for camera to register
        node_.provide_rpc("RegsiterSensorDevice",&_RegisterSensorDevice,this);
        for (unsigned int ii=0; ii<related_devices.size(); ii++) {
          related_devices.at(ii)->m_bHasAdvertised = true;
        }
      }
      /// GPS
//      else if(Device->m_sDeviceType=="GPS" && !Device->m_bHasAdvertised){
//        vector<SimDeviceInfo*> related_devices =
//            m_pSimDevices->GetAllRelatedDevices(Device->GetBodyName());
//        SimGPS* pGPS = (SimGPS*) related_devices.at(0);
//        pGPS->m_bDeviceOn = true;
//        node_.advertise(pGPS->GetDeviceName());
//      }
//      /// VICON
//      else if(static_cast<SimVicon*>(Device) != NULL){
//        SimVicon* pVicon = (SimVicon*) Device;
//        pVicon->m_bDeviceOn = true;
//        pVicon->Update();
//      }

      /*******************
       * SimControllers
       ********************/

      else if(Device->m_sDeviceType=="CarController"
              && !Device->m_bHasAdvertised){
        CarController* pCarCon = (CarController*) Device;
        node_.provide_rpc("RegisterControllerDevice",
                           &_RegisterControllerDevice, this);
        pCarCon->m_bHasAdvertised = true;
      }

//      else if(static_cast<SimpleController*>(Device) != NULL){
//        SimpleController* pSimpleCon = (SimpleController*) Device;
//        pSimpleCon->m_bDeviceOn = true;
//      }
    }
  }
}

////////////////////////////////////////////////////////////////////////
/// REGISTER AND DELETE DEVICES FROM THE SIMULATION

void NetworkManager::RegisterSensorDevice(RegisterNodeCamReqMsg& mRequest,
                                          RegisterNodeCamRepMsg & mReply){
  LOG(debug_level_) << "NodeCam asking for register in timestep "
                    << m_iTimeStep<<".";
  string sDeviceName = CheckURI(mRequest.uri());
  if(sDeviceName!="FALSE"){
    vector<SimDeviceInfo*> pDevices =
        m_pSimDevices->GetOnRelatedDevices(sDeviceName);

    /// TODO: Get all of the devices in this function!!

    SimCamera* pCam = (SimCamera*) pDevices.at(0);
    // For multiple-camera systems, we take the parameters from
    // the first camera.
    mReply.set_time_step(m_iTimeStep);
    mReply.set_regsiter_flag(1);
    mReply.set_channels(pDevices.size());
    mReply.set_width(pCam->m_nImgWidth);
    mReply.set_height(pCam->m_nImgHeight);
    m_iNodeClients = m_iNodeClients + 1;
    string sServiceName = GetFirstName(pCam->GetBodyName());
    if (node_.advertise(sServiceName)==true) {
      LOG(debug_level_) << "SUCCESS: Advertising " << sServiceName;
    } else {
      LOG(debug_level_) << "FAILURE: Advertising " << sServiceName;
      exit(-1);
    }
  }
  else{
    mReply.set_time_step(m_iTimeStep);
    mReply.set_regsiter_flag(0);
    LOG(debug_level_) << "HAL device named " << sDeviceName << " is invalid!";
  }
}

////////////////////////////////////////////////////////////////////////

void NetworkManager::RegisterControllerDevice(
    pb::RegisterControllerReqMsg& mRequest,
    pb::RegisterControllerRepMsg & mReply){
  string controller_name = CheckURI(mRequest.uri());
  if (controller_name!="FALSE") {
    LOG(debug_level_) << "Attempting to subscribe to "
                      << controller_name+"/"+controller_name;
    int subscribe_try = 0;
    while (node_.subscribe(controller_name+"/"+controller_name) != true &&
           subscribe_try<500) {
      LOG(debug_level_) << ".";
      usleep(100);
      subscribe_try++;
    }
    if (subscribe_try>=500) {
      LOG(debug_level_) << "FAILURE: Cannot subscribe to "<<controller_name;
    } else {
      LOG(debug_level_) << "We subscribed!";
      mReply.set_success(true);
    }
  }
}

////////////////////////////////////////////////////////////////////////
/// UPDATE AND PUBLISH INFO
////////////////////////////////////////////////////////////////////////

bool NetworkManager::UpdateNetwork(){
  m_iTimeStep++;
  /// -s WithoutNetwork
  if(m_sServerName == "WithoutNetwork"){
    return true;
  }
  /// -s WithoutStateKeeper
  ///  N.B. = A sensor must be published by its BODY NAME, called with
  ///         sensor->GetBodyName(). This is because one body can have
  ///         more than one device on it, and so we must account for them
  ///         all.
  ///         A controller must be called by its DEVICE NAME, called with
  ///         controller->GetDeviceName(). It doesn't have a physics body;
  ///         rather, it just controls one. It's purely a device.
  vector<SimDeviceInfo*> pDevices = m_pSimDevices->GetOnDevices();
  LOG(debug_level_) << "Num. devices: " << pDevices.size();
  for(unsigned int ii=0; ii<pDevices.size(); ii++){
    SimDeviceInfo* Device = pDevices.at(ii);
    ////////////////////
    // SENSORS
    //-- Camera
    if(Device->m_sDeviceType=="Camera" && !Device->m_bHasPublished){
      PublishSimCamBySensor(Device->GetBodyName());
    }
    //-- GPS
    else if(Device->m_sDeviceType=="GPS" && !Device->m_bHasPublished){
      PublishGPS(Device->GetBodyName());
    }
    ////////////////////
    // CONTROLLERS
    //-- CarController
    else if(Device->m_sDeviceType=="CarController" &&
            !Device->m_bHasPublished){
      ReceiveControllerInfo(Device->GetDeviceName());
    }
    Device->m_bHasPublished = false;
  }

  /// -s WithStateKeeper
  if (m_sServerName == "WithStateKeeper") {
    if (PublishRobotToStateKeeper() == false) {
      LOG(debug_level_) << "FAILURE: Cannot Publish Robot State To StateKeeper!"
                        << " You might be disconnected from the server...";
      return false;
    }
    if (ReceiveWorldFromStateKeeper() == false) {
      LOG(debug_level_) << "Cannot Receive World State from StateKeeper! "
                        << "You may be disconnected from the server... ";
      return false;
    }
  }
  return true;
}

////////////////////////////////////////////////////////////////////////
// Publish SimCamera by Sensor.
// We MUST MAKE SURE that the glGraph has been activated, or else there
// will be nothing to take a picture of!

bool NetworkManager::PublishSimCamBySensor(string sCamBodyName){
  pb::CameraMsg mCamImage;
  mCamImage.set_device_time(m_iTimeStep);
  vector<SimDeviceInfo*> pDevices =
      m_pSimDevices->GetOnRelatedDevices(sCamBodyName);
  for(unsigned int ii=0; ii<pDevices.size(); ii++){
    SimCamera* pSimCam = (SimCamera*) pDevices.at(ii);
    pSimCam->m_bHasPublished = true;

    ////////////
    // A grayscale image
    ////////////
    if(pSimCam->m_iCamType == SceneGraph::eSimCamLuminance){
      pb::ImageMsg *pImage = mCamImage.add_image();
      char* pImgbuf = (char*)malloc (pSimCam->m_nImgWidth *
                                     pSimCam->m_nImgHeight);
      if(pSimCam->capture(pImgbuf) == true){
        pImage->set_timestamp( m_iTimeStep);
        pImage->set_width( pSimCam->m_nImgWidth );
        pImage->set_height( pSimCam->m_nImgHeight );
        pImage->set_type(pb::PB_UNSIGNED_SHORT);
        pImage->set_format(pb::PB_LUMINANCE);
        pImage->set_data(pImgbuf);
        LOG(debug_level_) << "Published Greyscale";
      }
      else{
        return false;
      }
      free(pImgbuf);
    }
    ////////////
    // An RGB image
    ////////////
    else if(pSimCam->m_iCamType == SceneGraph::eSimCamRGB){
      pb::ImageMsg *pImage = mCamImage.add_image();
      char* pImgbuf= (char*)malloc (pSimCam->m_nImgWidth *
                                    pSimCam->m_nImgHeight * 3);
      if(pSimCam->capture(pImgbuf)==true){
        pImage->set_data(pImgbuf);
        pImage->set_timestamp( m_iTimeStep);
        pImage->set_width( pSimCam->m_nImgWidth );
        pImage->set_height( pSimCam->m_nImgHeight );
        pImage->set_type(pb::PB_UNSIGNED_BYTE);
        pImage->set_format(pb::PB_RGB);
        LOG(debug_level_) << "Published RGB";
      }
      else{
        return false;
      }
      free(pImgbuf);
    }
    ////////////
    // A depth image
    ////////////
    else if(pSimCam->m_iCamType == SceneGraph::eSimCamDepth){
      pb::ImageMsg *pImage = mCamImage.add_image();
      float* pImgbuf = (float*) malloc( pSimCam->m_nImgWidth *
                                        pSimCam->m_nImgHeight *
                                        sizeof(float) );
      if(pSimCam->capture(pImgbuf)==true){
        pImage->set_data((char*)pImgbuf);
        pImage->set_timestamp( m_iTimeStep);
        pImage->set_width( pSimCam->m_nImgWidth );
        pImage->set_height( pSimCam->m_nImgHeight );
        pImage->set_type(pb::PB_FLOAT);
        pImage->set_format(pb::PB_LUMINANCE);
        LOG(debug_level_) << "Published Depth";
      }
      else{
        return false;
      }
      free(pImgbuf);
    }
  }

  // Publish the info
  string sFirstName = GetFirstName(sCamBodyName);
  LOG(debug_level_) << "Camera handle to puublish: " << sFirstName;
  bool bStatus = node_.publish(sFirstName, mCamImage);
  if(!bStatus){
    LOG(ERROR) << "FAILURE: [" << m_sLocalSimName << "/" << sFirstName
               << "] cannot publish images"<<endl;
    return false;
  }
  LOG(debug_level_) << "SUCCESS: [" << m_sLocalSimName << "/" << sFirstName
                    << "] NodeCam published." <<endl;
  return true;
}

////////////////////////////////////////////////////////////////////////
// publish GPS by device name

bool NetworkManager::PublishGPS(string sDeviceName){
  Eigen::Vector3d pose;
  SimGPS* pGPS = (SimGPS*) m_pSimDevices->m_vSimDevices[sDeviceName];
  pGPS->GetPose(pose);
  GPSMsg mGPSMSg;
  mGPSMSg.set_time_step(m_iTimeStep);
  mGPSMSg.set_x(pose[0]);
  mGPSMSg.set_y(pose[1]);
  mGPSMSg.set_y(pose[2]);
  string sFirstName = GetFirstName(sDeviceName);
  LOG(debug_level_) << "Attempting to publish " << sFirstName
                    << ". x=" << pose[0]
                    << " y=" << pose[1] << " z=" << pose[2]
                    << ". Time step is " << mGPSMSg.time_step() << ".";
  bool bStatus=false;
  while(bStatus==false){
    bStatus=node_.publish(sFirstName, mGPSMSg);
    if( bStatus==false){
      LOG(ERROR) << "FAILURE: Could not publish GPS data.";
    }
  }
  LOG(debug_level_) << "SUCCESS: Published GPS data";
  return true;
}

////////////////////////////////////////////////////////////////////////
// Receive information from all controllers hooked up to HAL.
// (we may have more than one controller here.)

bool NetworkManager::ReceiveControllerInfo(string sDeviceName){
  SimDeviceInfo* pDevice = m_pSimDevices->m_vSimDevices[sDeviceName];
  /// TODO: There's a naming issue here; we have to duplicate the name
  /// to register correctly.
  string sServiceName = GetFirstName(sDeviceName)+"/"+GetFirstName(sDeviceName);
  LOG(debug_level_) << "Attempting to connect to " << sServiceName;

  // ASYNCHRONIZED PROTOCOL: The while loop doesn't wait forever to receive
  //   a command
  // SYNCHRONIZED PROTOCOL: ...we wait.
  bool sync = false;
  LOG(debug_level_) << "Syncing protocol is " << sync;
  int max_iter;
  sync ? max_iter = 1e7 : max_iter = 100;

  // Car Controller
  if(pDevice->m_sDeviceType=="CarController"){
    CarController* pCarController = (CarController*) pDevice;
    pb::VehicleMsg Command;
    int n = 0;
    while(node_.receive(sServiceName, Command)==false
          && n < max_iter){
      LOG(debug_level_) << ".";
      n++;
    }
    if(n==max_iter){
      LOG(debug_level_) << "No command";
      return false;
    }
    else{
      LOG(debug_level_) << "Got command!";
      pCarController->UpdateCommand(Command);
      pCarController->m_bHasPublished = true;
      return true;
    }
  }

  // Simple Controller (aka Camera Body control?)
  else if(pDevice->m_sDeviceType=="SimpleController"){
    SimpleController* pSimpController = (SimpleController*) pDevice;
    pb::PoseMsg Command;
    if(node_.receive( sServiceName, Command)==true){
      pSimpController->UpdateCommand(Command);
      return true;
    }
    else{
      LOG(ERROR) << "FAILURE: Couldn't get the command for the"
                 << " Simple Controller";
      return false;
    }
  }
  return true;
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
    if(m_verbosity != 0){cout<<"[NetworkManager/RegisterRobot]"<<
                               " Skip due to WithoutNetwork mode"<<endl;}
  }
  else if(m_sServerName == "WithoutStateKeeper"){
    if(m_verbosity != 0){cout<<"[NetworkManager/RegisterRobot]"<<
                               " Skip due to WithoutStateKeeper mode"<<endl;}
  }

  // We have a StateKeeper! Go publish. Now.
  else if(m_sServerName == "WithStateKeeper"){
    m_pRobotsManager = pRobotsManager;
    node_.advertise("RobotState");
    bool bStatus = RegisterWithStateKeeper();
    if(bStatus == false){
      if(m_verbosity != 0){cout<<"[NetworkManager] Cannot register LocalSim '"<<
                                 m_sLocalSimName<<"' in "<<m_sServerName<<
                                 ". Please make sure "<<m_sServerName<<
                                 " is running!"<<endl;}
      return false;
    }
    // TODO: Allow these methods; not sure if they work yet, though.
    node_.provide_rpc("AddRobotByURDF",&_AddRobotByURDF, this);
    node_.provide_rpc("DeleteRobot",&_DeleteRobot, this);
    if(m_verbosity != 0){cout<<"[NetworkManager] Init Robot Network "<<
                               m_sLocalSimName<<
                               " for statekeeper success."<<endl;}
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
  if( node_.subscribe(sServiceName) == false ){
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
  if(node_.call_rpc(sServiceName, mRequest, mReply) == true &&
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
  // Don't let anyone touch the shared resource table...
  std::lock_guard<std::mutex>;

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
  std::lock_guard<std::mutex>;

  // 1. Set robot name and time step info
  RobotFullStateMsg mRobotFullState;
  mRobotFullState.set_robot_name(
        m_pRobotsManager->GetMainRobot()->GetRobotName());
  // mark it as the lastest robot state by time_step +1.
  mRobotFullState.set_time_step(m_iTimeStep);

  // 2. Set body state info
  vector<string> vAllBodyFullName =
      m_pRobotsManager->GetMainRobot()->GetAllBodyName();

  // TODO: Fix this implementation.

//  for (unsigned int i=0;i!=vAllBodyFullName.size();i++)
//  {
//    string sBodyName = vAllBodyFullName[i];
//    // prepare pose info
//    Eigen::Vector3d eOrigin =
//        m_pRobotsManager->m_Scene.m_Phys.GetEntityOrigin(sBodyName);
//    Eigen::Matrix3d eBasis =
//        m_pRobotsManager->m_Scene.m_Phys.GetEntityBasis(sBodyName);

//    // prepare veloicty info
//    Eigen::Vector3d eLinearV =
//        m_pRobotsManager->m_Scene.m_Phys.GetEntityLinearVelocity(sBodyName);
//    Eigen::Vector3d eAngularV =
//        m_pRobotsManager->m_Scene.m_Phys.GetEntityAngularVelocity(sBodyName);

//    // set pose info
//    BodyStateMsg* mBodyState = mRobotFullState.add_body_state();
//    mBodyState->set_body_name(sBodyName);
//    mBodyState->mutable_origin()->set_x(eOrigin[0]);
//    mBodyState->mutable_origin()->set_y(eOrigin[1]);
//    mBodyState->mutable_origin()->set_z(eOrigin[2]);

//    mBodyState->mutable_basis()->set_x11(eBasis(0,0));
//    mBodyState->mutable_basis()->set_x12(eBasis(0,1));
//    mBodyState->mutable_basis()->set_x13(eBasis(0,2));
//    mBodyState->mutable_basis()->set_x21(eBasis(1,0));
//    mBodyState->mutable_basis()->set_x22(eBasis(1,1));
//    mBodyState->mutable_basis()->set_x23(eBasis(1,2));
//    mBodyState->mutable_basis()->set_x31(eBasis(2,0));
//    mBodyState->mutable_basis()->set_x32(eBasis(2,1));
//    mBodyState->mutable_basis()->set_x33(eBasis(2,2));

//    // set velocity
//    mBodyState->mutable_linear_velocity()->set_x(eLinearV[0]);
//    mBodyState->mutable_linear_velocity()->set_y(eLinearV[1]);
//    mBodyState->mutable_linear_velocity()->set_z(eLinearV[2]);

//    mBodyState->mutable_angular_velocity()->set_x(eAngularV[0]);
//    mBodyState->mutable_angular_velocity()->set_y(eAngularV[1]);
//    mBodyState->mutable_angular_velocity()->set_z(eAngularV[2]);
//  }

  // 4. Publish robot state
  bool bStatus=false;
  while (bStatus==false)
  {
    bStatus=node_.publish( "RobotState", mRobotFullState);
    if(bStatus==true)
    {
      if(m_verbosity!=0)
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
  std::lock_guard<std::mutex>;
  WorldFullStateMsg ws;
  string sServiceName = m_sServerName + "/WorldState";
  // wait until we get the lastest world state
  int iMaxTry=50;
  bool bStatus=false;
  while(bStatus==false){
    if(node_.receive(sServiceName, ws )==true && ws.time_step() >=
       m_pRobotsManager->m_WorldFullState.time_step()){
      // update world state in robots manager.
      m_pRobotsManager->UpdateWorldFullState(ws);
      m_pRobotsManager->ApplyWorldFullState();
      m_iTimeStep = ws.time_step();
      bStatus=true;
      if(m_verbosity!=0){
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
