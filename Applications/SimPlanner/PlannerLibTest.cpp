#include "PlannerLibTest.h"

#include <thread>

/// CONSTRUCTOR
PlannerLibTest::PlannerLibTest(){
  params_file_name_ =
      "/Users/Trystan/Code/CarPlanner/MochaGui/gui_params.csv";
      // "/Users/Trystan/Code/simba/Applications/SimPlanner/raycast_params.csv";


}

/// DESTRUCTOR
PlannerLibTest::~PlannerLibTest(){
}

/////////////////////////////////////////////

void PlannerLibTest::Init(HeightmapShape* heightmap_data){
  heightmap_data_ = heightmap_data;
  InitGoals();
  InitMesh();
}

/////////////////////////////////////////////
pb::BVP_policy PlannerLibTest::StartPolicy(){
  pb::BVP_policy policy = SampleTrajectory();
  return policy;
}
/////////////////////////////////////////////

void PlannerLibTest::InitGoals(){
  // <x, y, theta, vel>
  start_.push_back(1);
  start_.push_back(-.5);
  start_.push_back(0);
  start_.push_back(1);
  goal_.push_back(1);
  goal_.push_back(0);
  goal_.push_back(0);
  goal_.push_back(2);
  // Now populate the start and goal parameters.
  // X, Y, yaw, and velocity... that should be it.
  Eigen::Matrix4d eigen_start;
  Eigen::Vector6d eigen_cart;
  eigen_cart<<start_[0], start_[1], .5, 0, start_[2], 0;
  eigen_start = _Cart2T(eigen_cart);
  Eigen::Matrix4d eigen_goal;
  eigen_cart<<goal_[0], goal_[1], .5, 0, goal_[2], 0;
  eigen_goal = _Cart2T(eigen_cart);
  // Populate the VehicleStates
  start_state_ = VehicleState(Sophus::SE3d(eigen_start), start_[3], 0);
  goal_state_ = VehicleState(Sophus::SE3d(eigen_goal), goal_[3], 0);
}

//////////////////

bool PlannerLibTest::InitMesh(){
  // We don't want to reinitialize if we have the same map, after all.
  bullet_heightmap* map = new bullet_heightmap(heightmap_data_);
  btVector3 dMin(DBL_MAX,DBL_MAX,DBL_MAX);
  btVector3 dMax(DBL_MIN,DBL_MIN,DBL_MIN);
  CarParameters::LoadFromFile(params_file_name_, m_VehicleParams);
  // MAKE SURE YOU ADD ENOUGH FREAKIN' WORLDS TO THE CAR MODEL.
  // 11 should do it.
  Eigen::Matrix4d eigen_start;
  Eigen::Vector6d eigen_cart;
  eigen_cart<<start_[0], start_[1], 2, 0, start_[2], 0;
  eigen_start = _Cart2T(eigen_cart);
  car_model_ = new BulletCarModel();
  btTransform localTrans;
  localTrans.setIdentity();
  localTrans.setOrigin(btVector3(1,.5,.9));
  car_model_->Init(heightmap_data_->row_count_,
                   heightmap_data_->col_count_,
                   heightmap_data_->x_data_,
                   heightmap_data_->y_data_,
                   heightmap_data_->z_data_,
                   localTrans, dMin, dMax, m_VehicleParams, 11);
  SceneGraph::GLHeightmap* new_map =
      new SceneGraph::GLHeightmap(heightmap_data_->x_data_,
                                  heightmap_data_->y_data_,
                                  heightmap_data_->z_data_,
                                  heightmap_data_->row_count_,
                                  heightmap_data_->col_count_);
  planner_gui_.Init( new_map );
  // m_GLDebugDrawer.Init(car_model_);
  // planner_gui_.AddGLObject(&m_GLDebugDrawer);
  // Setting the scale of our car
  // Eigen::Vector3d vBodyDim, vWheelDim, vWheelScale, vBodyScale;
  // vBodyDim << 0.901 ,0.338, 0.279;
  // vWheelDim << 0.4313, 0.145, 0.145;
  // vWheelScale<<2*m_VehicleParams[CarParameters::WheelRadius]/vWheelDim(1),
  //     2*m_VehicleParams[CarParameters::WheelRadius]/vWheelDim(2),
  //     m_VehicleParams[CarParameters::WheelWidth]/vWheelDim(0);
  // vBodyScale<<m_VehicleParams[CarParameters::WheelBase]/vBodyDim(1),
  //     1.5*m_VehicleParams[CarParameters::Width]/vBodyDim(0),
  //     m_VehicleParams[CarParameters::Height]/vBodyDim(2);
  // // planner_gui_.AddCar(vBodyScale, vWheelScale);
  planner_gui_.AddCar(m_VehicleParams[CarParameters::WheelBase],
                      m_VehicleParams[CarParameters::Width],
                      m_VehicleParams[CarParameters::Height],
                      m_VehicleParams[CarParameters::WheelRadius],
                      m_VehicleParams[CarParameters::WheelWidth]);
  /////
  LOG(INFO) << "Init our mesh!";
  GroundStates();
  LOG(INFO) << start_state_.m_dTwv.matrix();
  LOG(INFO) << goal_state_.m_dTwv.matrix();
  // Render what we see
  planner_gui_.AddWaypoint(start_state_);
  planner_gui_.AddWaypoint(goal_state_);
}

//////////////////////////////////////////////////

void PlannerLibTest::GroundStates(){
  // Ground the start point.
  Eigen::Vector3d dIntersect, normal;
  Sophus::SE3d pose = start_state_.m_dTwv;
  if(car_model_->RayCast(pose.translation(), GetBasisVector(pose,2)*10,
                         dIntersect, true, 0)){
    dIntersect(2) = dIntersect(2)+.15;
    start_state_.m_dTwv.translation() = dIntersect;
    LOG(INFO) << pose.translation();
    if(car_model_->RayCastNormal(pose.translation(),
                                 GetBasisVector(start_state_.m_dTwv,2),
                                 normal, 0)){
      Eigen::Quaternion<double> quatRot(Eigen::AngleAxis<double>(
          start_state_.GetTheta(), normal));
      Eigen::Matrix3d rotMat = quatRot*start_state_.m_dTwv.rotationMatrix();
      start_state_.m_dTwv =
          Sophus::SE3d(rotMat, start_state_.m_dTwv.translation());
      LOG(INFO) << "Setting start point to ground...";
    }

    // Ground the goal point
    pose = goal_state_.m_dTwv;
    if(car_model_->RayCast(pose.translation(), GetBasisVector(pose,2)*30,
                           dIntersect, true, 0)){
      goal_state_.m_dTwv.translation() = dIntersect;
      if(car_model_->RayCastNormal(pose.translation(),
                                   GetBasisVector(pose,2)*10,
                                   dIntersect, 0)){
        normal = normal.normalized();
        Eigen::Quaternion<double> quatRot(Eigen::AngleAxis<double>(
            goal_state_.GetTheta(), normal));
        Eigen::Matrix3d rotMat = quatRot*goal_state_.m_dTwv.rotationMatrix();
        goal_state_.m_dTwv =
            Sophus::SE3d(rotMat, goal_state_.m_dTwv.translation());
        LOG(INFO) << "Setting goal point to ground...";
      }
    }
  }
}

/////////////////////////////////////////////

//Finds the fastest path between two
pb::BVP_policy PlannerLibTest::SampleTrajectory(){
  bool success = false;
  int count = 0;
  ApplyVelocitesFunctor5d func(car_model_, Eigen::Vector3d::Zero(), NULL);
  func.SetNoDelay(true);
  MotionSample sample;
  car_model_->SetState(0, start_state_);
  // while(1){
  //   VehicleState state;
  //   Eigen::Vector3d torques;
  //   torques<<0,0,0;
  //   ControlCommand command(1, 0, torques, 1.0/30.0, 1.7);
  //   // car_model_->UpdateState(0, command);
  //   car_model_->GetVehicleState(0, state);
  //   planner_gui_.SetCarState(0, state, true);
  //   planner_gui_.Render();
  //   std::this_thread::sleep_for(std::chrono::milliseconds(50));
  // }
  // TODO: Get the waypoints working.
  // It's the easiest way to plan

  GLWayPoint* a = &planner_gui_.GetWaypoint(0)->m_Waypoint;
  GLWayPoint* b = &planner_gui_.GetWaypoint(1)->m_Waypoint;
  VehicleState st(Sophus::SE3d(a->GetPose4x4_po()),a->GetVelocity(),0);
  VehicleState gl(Sophus::SE3d(b->GetPose4x4_po()),b->GetVelocity(),0);
  LocalProblem problem(&func, st, gl, 1.0/30.0);
  m_snapper.InitializeLocalProblem(problem, 0, NULL, eCostPoint);
  while(!success && count<20){
    success = m_snapper.Iterate(problem);
    m_snapper.SimulateTrajectory(sample,problem,0,true);

    // Render what we see
    for (int ii=0; ii<sample.m_vStates.size(); ii++){
      planner_gui_.SetCarState(0, sample.m_vStates.at(ii), true);
      LOG(INFO) << std::endl
                << sample.m_vStates.at(ii).m_dTwv.matrix();
      planner_gui_.Render();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    count++;
  }
  if (problem.m_bInertialControlActive) {
    m_snapper.CalculateTorqueCoefficients(problem, &sample);
    m_snapper.SimulateTrajectory(sample,problem,0,true);
  }
  VehicleState last_vehicle_state = sample.GetLastPose();
  pb::BVP_policy policy;
  // We have to get all of the commands over the time period described.
  // And the start/ goal state. That's helpful too.
  for(unsigned int ii=0; ii<sample.m_vCommands.size(); ii++){
    ControlCommand Comm = sample.m_vCommands.at(ii);
    policy.add_force(Comm.m_dForce);
    policy.add_phi(Comm.m_dPhi);
    policy.add_time(Comm.m_dT);
  }
  for(unsigned int jj=0; jj<4; jj++){
    policy.add_start_param(start_.at(jj));
    policy.add_goal_param(goal_.at(jj));
  }
  if(count==100){
    LOG(INFO) << "We're done with our policy search!!";
    LOG(INFO) << "We hit the maximum number of iterations...";
  } else {
    LOG(INFO) << "We're done with our policy search!!";
  }
  policy.set_tau(m_nTau);
  return policy;
}

//////////////////

std::string PlannerLibTest::GetNumber(std::string name){
  std::size_t found = name.find("m");
  if(found!=std::string::npos){
    return name.substr(found+1);
  }
}

/***********************************
 * THE MAIN LOOP
 * This is a test, so we're only going to do ONE path through
 * the simplest type of a terrain, the flat plane. Get the commands
 * back from this simulation, and start a separate Node process
 * that sends these commands to LocalSim in SimBA. If that seems to
 * work, then start the rest.
 * Order of operations:
 * 1. Start MATLAB and PlannerLibTest. Send mesh plan to MATLAB,
 *    start the simulation, get a plan. Make the plan into an array
 *    that can be easily passed piecemeal to SimBA [DONE]
 * 2. Once we have this vector, start the Node connection. Through
 *    a separate terminal, start SimBA's LocalSim. LocalSim is going
 *    to grab the same mesh from MATLAB (maybe pass through PlannerLibTest?)
 *    Once this is done, start the CarController. Once that is
 *    connected to the Node in PlannerLibTest, then start sending commmands.
 * You see why this is a hard thing to test. We need to start from MATLAB in
 * case that's where the problem lies, like the mesh is created
 * sideways or something.
 **********************************/

int main(int argc, char** argv){

  // Put a PlannerGui here, so that we can see what we're doing.

  PlannerLibTest* sim = new PlannerLibTest();
  std::string name = "Sim0";
  if (argc==2) {
    name = argv[1];
  }
  sim->sim_planner_name_ = name;
  std::string number = sim->GetNumber(sim->sim_planner_name_);
  URDF_Parser* parser = new URDF_Parser(0);
  // 1. Read URDF files.
  XMLDocument world_xml;
  const string& world_urdf_path =
      "/Users/Trystan/Code/simba/urdf/Worlds/world_localplanner.xml";
  GetXMLdoc(world_urdf_path, world_xml);
  HeightmapShape* heightmap_data = parser->GetMeshData(world_xml);
  sim->Init(heightmap_data);
  // 2. Solve for the path using PlannerLib
  pb::BVP_policy policy;
  policy = sim->StartPolicy();
  // This is our solved policy, which we will send through the
  // interwebs.
  // 3. Start looking for the car (similar to KeyboardCommander)
  KeyCarController KeyCar("NodeCar:[name=VehicleController,sim=Ricky]//",
                          policy);
  while (KeyCar.ApplyCommands()){
    // 4. Drive our car to the EDGE OF SPACE!!!!!!!1!!
    //   policy.add_start_param(start_.at(jj));
    //   policy.add_goal_param(goal_.at(jj));
  }
  // 5. Measure the delta between the where the vehicle is and where
  //    it's supposed to be. The policy holds the destination already.
  std::vector<double> start_param, goal_param;
  for (int ii=0; ii<policy.start_param_size(); ii++) {
    start_param.push_back(policy.start_param(ii));
    goal_param.push_back(policy.goal_param(ii));
  }
  std::cout<<"Done with all of our shenanigans!"<<std::endl;
}
