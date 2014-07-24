/************************************************************
 *
 * STATEKEEPER FUNCTIONS
 *
 ***********************************************************/

// Used to commmunicate with the StateKeeper, if it's initialized.
bool StateKeeper::RegisterRobot(RobotsManager* pRobotsManager) {
  robot_manager_ = pRobotsManager;
  // Check if we need to connect to StateKeeper.
  if (server_name_ == "WithoutNetwork") {
    LOG(debug_level_) << "Skip due to WithoutNetwork mode";
  } else if (server_name_ == "WithoutStateKeeper") {
    LOG(debug_level_) << "Skip due to WithoutStateKeeper mode";
  }
  // We have a StateKeeper! Go publish. Now.
  else if (server_name_ == "WithStateKeeper") {
    node_.advertise("RobotState");
    bool bStatus = RegisterWithStateKeeper();
    if (bStatus == false) {
      LOG(debug_level_) << "Cannot register LocalSim '"
                        << local_sim_name_ << "' in " << server_name_
                        << ". Please make sure " << server_name_
                        << " is running!";
      return false;
    }
    // TODO: Allow these methods; not sure if they work yet, though.
    node_.provide_rpc("AddRobotByURDF",&_AddRobotByURDF, this);
    node_.provide_rpc("DeleteRobot",&_DeleteRobot, this);
    LOG(debug_level_) << "Init Robot Network "
                      << local_sim_name_ << " for statekeeper success.";
  }
  return true;
}


////////////////////////////////////////////////////////////////////////
/// REGISTER AND DELETE ROBOTS/DEVICES FROM STATEKEEPER
/// 1. subscribe to WorldState Topic.
/// 2. Send Robot's URDF file to StateKeeper.
/// 3. Receive init pose for Robot.

bool StateKeeper::RegisterWithStateKeeper()
{
  // 1. Subscribe to StateKeeper World state topic
  string sServiceName = server_name_+"/WorldState";
  if (!node_.subscribe(sServiceName)) {
    LOG(WARNING) << "FAILURE: Cannot subscribe to "<<sServiceName;
    return false;
  }

  // 2. prepare URDF file to StateKeeper
  //  - Get Robot URDF (.xml) file
  //  - Set request msg
  SimRobot* pSimRobot = robot_manager_->sim_robots_map_.begin()->second;
  tinyxml2::XMLPrinter printer;
  pSimRobot->GetRobotURDF()->Accept(&printer);
  RegisterLocalSimReqMsg mRequest;
  string sRobotName = pSimRobot->GetRobotName();
  mRequest.set_proxy_name(local_sim_name_);
  mRequest.mutable_urdf()->set_robot_name(sRobotName);
  mRequest.mutable_urdf()->set_xml(printer.CStr());

  // 3. Call StateKeeper to register robot: service name, request_msg,
  // reply_msg. Reply message must be empty.
  RegisterLocalSimRepMsg mReply;
  sServiceName = server_name_ + "/RegisterLocalSim";
  if (node_.call_rpc(sServiceName, mRequest, mReply) == true &&
      mReply.robot_name() == sRobotName) {
    Vector6Msg  mInitRobotState = mReply.init_pose();
    LOG(debug_level_) << "Set URDF to StateKeeper success";
    // 3.1 init time step. this is very important step.
    timestep_ = mReply.time_step();
    // 3.2 init pose state of my robot.
    Eigen::Vector6d ePose;
    ePose<<mInitRobotState.x(), mInitRobotState.y(), mInitRobotState.z(),
        mInitRobotState.p(), mInitRobotState.q(), mInitRobotState.r();

    LOG(debug_level_) << "Robot register success!"
                      << " Get init robot state as x: " << ePose[0]
                      << " y: " << ePose[1]
                      << " z: " << ePose[2]
                      << " p: " << ePose[3]
                      << " q: " << ePose[4]
                      << " r: " << ePose[5]
                      << ". in Time step: " << timestep_;

    // 3.3 build other robots that already in StateKeeper in our proxy.
    // Read initial pose of them from URDF. This is a trick as their real
    // pose will be set quickly when we sync worldstate message.
    LOG(debug_level_) << "Try to init " << mReply.urdf_size()
                      << " previous players.";
    for (int i = 0; i != mReply.urdf_size(); i++) {
      // prepare urdf
      const URDFMsg& urdf = mReply.urdf(i);
      string sFullName = urdf.robot_name();
      string sLastName = GetLastName(sFullName);
      std::shared_ptr<tinyxml2::XMLDocument> doc =
          std::make_shared<tinyxml2::XMLDocument>();
      doc->Parse(urdf.xml().c_str());
      // create previous robot
      URDFParser* parse = new URDFParser(debug_level_);
      std::shared_ptr<SimRobot> robot = std::make_shared<SimRobot>();
      parse->ParseRobot(doc, sLastName, robot);
      robot_manager_->ImportSimRobot(*robot);

      // TODO: How to add this back into the scene...
      LOG(debug_level_) << "SUCCESS: Init previous player " << sFullName
                        << ". Last Name " << sLastName;
    }
    return true;
  } else {
    LOG(WARNING) << "FAILURE: LocalSim register";
    return false;
  }
}

////////////////////////////////////////////////////////////////////////
/// Add a new robot to the Sim (through StateKeeper)

void StateKeeper::AddRobotByURDF(LocalSimAddNewRobotReqMsg& mRequest,
                                    LocalSimAddNewRobotRepMsg& mReply)
{
  // set reply message for StateKeeper
  mReply.set_message("AddNewRobotSuccess");

  // Create New Robot Base on URDF File in my proxy
  tinyxml2::XMLDocument doc;
  doc.Parse(mRequest.urdf().xml().c_str());
  string sNewAddRobotName = mRequest.robot_name();
  string sProxyNameOfNewRobot= GetRobotLastName(sNewAddRobotName);

  Eigen::Vector6d ePose;
  ePose<<mRequest.mutable_init_pose()->x(), mRequest.mutable_init_pose()->y(),
      mRequest.mutable_init_pose()->z(), mRequest.mutable_init_pose()->p(),
      mRequest.mutable_init_pose()->q(), mRequest.mutable_init_pose()->r();

  // add new robot in proxy
  URDFParser* parse = new URDFParser(debug_level_);
  SimRobot* robot = new SimRobot();
  parse->ParseRobot(doc, *robot, sProxyNameOfNewRobot);
  robot_manager_->ImportSimRobot(*robot);
  LOG(debug_level_) << "SUCCESS: Added new robot "
                    << mRequest.robot_name();
}

////////////////////////////////////////////////////////////////////////
/// Delete an existing robot from the Sim (through StateKeeper)

void StateKeeper::DeleteRobot(LocalSimDeleteRobotReqMsg& mRequest,
                                 LocalSimDeleteRobotRepMsg& mReply) {
  // Don't let anyone touch the shared resource table...
  std::lock_guard<std::mutex> lock(statekeeper_mutex_);

  // set reply message for StateKeeper
  mReply.set_message("DeleteRobotSuccess");

  // delete robot in our proxy
  string sRobotName = mRequest.robot_name();
  robot_manager_->DeleteRobot(sRobotName);
  LOG(debug_level_) << "SUCCESS: Deleted Robot "
                    << sRobotName;
}

////////////////////////////////////////////////////////////////////////
/// Sync WorldState by
/// (1) Publishing main robot's state and
/// (2) Receiving the world state
/// We read the robot state via a vicon device. This state includes the robot
/// pose, state, and current command.
bool StateKeeper::PublishRobotToStateKeeper() {
  // don't let anyone touch the shared resource table...
  std::lock_guard<std::mutex> lock(statekeeper_mutex_);

  // 1. Set robot name and time step info
  RobotFullStateMsg mRobotFullState;
  mRobotFullState.set_robot_name(
      robot_manager_->GetMainRobot()->GetRobotName());
  // mark it as the lastest robot state by time_step +1.
  mRobotFullState.set_time_step(timestep_);

  // 2. Set body state info
  vector<string> vAllBodyFullName =
      robot_manager_->GetMainRobot()->GetAllBodyName();

  // TODO: Fix this implementation.

  //  for (unsigned int i=0;i!=vAllBodyFullName.size();i++)
  //  {
  //    string sBodyName = vAllBodyFullName[i];
  //    // prepare pose info
  //    Eigen::Vector3d eOrigin =
  //        robot_manager_->m_Scene.m_Phys.GetEntityOrigin(sBodyName);
  //    Eigen::Matrix3d eBasis =
  //        robot_manager_->m_Scene.m_Phys.GetEntityBasis(sBodyName);

  //    // prepare veloicty info
  //    Eigen::Vector3d eLinearV =
  //        robot_manager_->m_Scene.m_Phys.GetEntityLinearVelocity(sBodyName);
  //    Eigen::Vector3d eAngularV =
  //        robot_manager_->m_Scene.m_Phys.GetEntityAngularVelocity(sBodyName);

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
  bool bStatus = false;
  while (bStatus == false) {
    bStatus = node_.publish("RobotState", mRobotFullState);
    if (bStatus==true) {
      LOG(debug_level_) << "Publish " << local_sim_name_
                        << " State to Statekeeper success. Publish Timestep is "
                        << timestep_;
      return true;
    } else {
      LOG(WARNING) << "ERROR: Publishing RobotState Fail.";
      return false;
    }
  }
  return true;
}

////////////////////////////////////////////////////////////////////////

bool StateKeeper::ReceiveWorldFromStateKeeper() {
  std::lock_guard<std::mutex> lock(statekeeper_mutex_);
  WorldFullStateMsg ws;
  string sServiceName = server_name_ + "/WorldState";
  // wait until we get the lastest world state
  int iMaxTry=50;
  bool bStatus=false;
  while (bStatus==false) {
    if (node_.receive(sServiceName, ws )==true && ws.time_step() >=
        robot_manager_->world_state_.time_step()) {
      // update world state in robots manager.
      robot_manager_->UpdateWorldFullState(ws);
      robot_manager_->ApplyWorldFullState();
      timestep_ = ws.time_step();
      bStatus=true;
      LOG(debug_level_) << "Update World state success! Size is "
                        << ws.robot_state_size()
                        << ". Time Step for world state is " << ws.time_step();
    } else if (bStatus == false && iMaxTry!=0) {
      usleep(50000);
      iMaxTry--;
    } else {
      LOG(WARNING) << "Update World state fail!";
      return false;
    }
  }
  return true;
}
