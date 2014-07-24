#include <Managers/RobotsManager.h>

////////////////////////////////////////////////////////////////////////
/// INITIALIZE the RobotsManager
////////////////////////////////////////////////////////////////////////

bool RobotsManager::Init(const string& sim_name, const ModelGraphBuilder& scene,
                         SimRobot& sim_robot,
			 const string& statekeeper_option) {
  scene_ = scene;
  sim_name_ = sim_name;
  if (statekeeper_option == "WithoutStateKeeper" ||
     statekeeper_option == "WithoutNetwork") {
    statekeeper_option_ = false;
  } else {
    statekeeper_option_ = true;
  }
  return ImportSimRobot(sim_robot);
}

///////
bool RobotsManager::ImportSimRobot(SimRobot& sim_robot) {
  if (sim_robot.GetStateKeeperStatus() == true) {
    //  Ummm
  }
  //  check if we should save name of this robot as save main robot name
  if (sim_robots_map_.size() == 0) {
    main_robot_name_ = sim_robot.GetRobotName();
  }
  sim_robots_map_.insert(pair<string, SimRobot*>(sim_robot.GetRobotName(),
                                                 &sim_robot));
  return true;
}

///////
// Delete the robot
// THE BELOW WILL NOT HAPPEN WITH A RAYCAST_VEHICLE
void RobotsManager::DeleteRobot(string robot_name) {
  SimRobot* pSimRobot = GetRobot(robot_name);
  if (pSimRobot->GetRobotName() == robot_name) {
    GetRobot(robot_name)->~SimRobot();
    std::map<string, SimRobot*>::iterator iter =
        sim_robots_map_.find(robot_name);
    sim_robots_map_.erase(iter);
    //  cout<<"[RobotsManager/DeleteRobot] Delete Robot :"<<robot_name<<
    // " success. Num of robot we have now is "<<sim_robots_map_.size()<<endl;
  } else {
    //  cout<<"[RobotManager/DeleteRobot] Cannot find robot "<<robot_name<<endl;
  }
}

////////////////////////////////////////////////////////////////////////

// Update the full world state: include all poses, commands and velocity
// information of all bodies creating the main robot from each LocalSim

void RobotsManager::UpdateWorldFullState(WorldFullStateMsg worldfullstate) {
  world_state_ = worldfullstate;
}

////////////////////////////////////////////////////////////////////////

// Draw all players on the screen
void RobotsManager::ApplyWorldFullStateOnAllPlayers() {
  DrawAllRobotsPoseAxis();
  ApplyWorldFullState();
}

////////////////////////////////////////////////////////////////////////

// Simply apply the poses of all main robots from all other LocalSims.
// DO NOT apply the pose of this LocalSim's main robot.

void RobotsManager::ApplyWorldFullState() {
  for (int i = 0; i != world_state_.robot_state_size(); i++) {
    RobotFullStateMsg mRobotState =  world_state_.robot_state(i);
    //  only apply state of other robots
    string robot_name = mRobotState.robot_name();

    if (robot_name != GetMainRobot()->GetRobotName()) {
      //  get and apply the state of robot's all body
      for (int j = 0; j!= mRobotState.mutable_body_state()->size(); j++) {
        BodyStateMsg* mBodyState = mRobotState.mutable_body_state(j);
        string sBodyName = mBodyState->body_name();

        //  get velocity
        Eigen::Vector3d eLinearVelocity;
        eLinearVelocity << mBodyState->linear_velocity().x(),
            mBodyState->linear_velocity().y(),
            mBodyState->linear_velocity().z();
        Eigen::Vector3d eAngularVelocity;
        eAngularVelocity << mBodyState->angular_velocity().x(),
            mBodyState->angular_velocity().y(),
            mBodyState->angular_velocity().z();

        //  get origin
        Eigen::Vector3d eOrigin;
        eOrigin << mBodyState->origin().x(),
            mBodyState->origin().y(), mBodyState->origin().z();

        //  get basis
        Matrix33Msg origin = mBodyState->basis();
        Eigen::Matrix3d mBasis;
        mBasis<<origin.x11(), origin.x12(), origin.x13(),
            origin.x21(), origin.x22(), origin.x23(),
            origin.x31(), origin.x32(), origin.x33();

        //  TODO: Make this happen. But not right now.

        //       //  apply in bullet engine
        //       scene_.m_Phys.SetEntityOrigin(sBodyName , eOrigin);
        //       scene_.m_Phys.SetEntityBasis(sBodyName , mBasis);
        //       scene_.m_Phys.SetEntityLinearvelocity(sBodyName,
        //                                             eLinearVelocity);
        //       scene_.m_Phys.SetEntityAngularvelocity(sBodyName,
        //                                              eAngularVelocity);
      }
    }
  }

}

////////////////////////////////////////////////////////////////////////

void RobotsManager::DrawAllRobotsPoseAxis() {
  //  Fill this in later
}

////////////////////////////////////////////////////////////////////////
SimRobot* RobotsManager::GetMainRobot() {
  SimRobot* pSimRobot = sim_robots_map_.find(main_robot_name_)->second;
  return pSimRobot;
}

////////////////////////////////////////////////////////////////////////
SimRobot* RobotsManager::GetRobot(string robot_name) {
  SimRobot* pSimRobot = sim_robots_map_.find(robot_name)->second;
  return pSimRobot;
}
