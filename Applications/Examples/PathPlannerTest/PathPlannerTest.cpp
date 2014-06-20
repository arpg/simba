#include "PathPlannerTest.h"

#include <thread>

/// CONSTRUCTOR
PathPlannerTest::PathPlannerTest(){
  params_file_name_ =
      "/Users/Trystan/Code/simba/Applications/Examples/PathPlannerTest/gui_params.csv";
}

/// DESTRUCTOR
PathPlannerTest::~PathPlannerTest(){
}

/////////////////////////////////////////////

void PathPlannerTest::Init(HeightmapShape* heightmap_data,
                           std::vector<double> start,
                           std::vector<double> goal){
  heightmap_data_ = heightmap_data;
  start_ = start;
  goal_ = goal;
  InitGoals();
  InitMesh();
}

/////////////////////////////////////////////

   void PathPlannerTest::InitGoals(){
  // Now populate the start and goal parameters.
  // X, Y, yaw, and velocity... that should be it.
  Eigen::Matrix4d eigen_start;
  Eigen::Vector6d eigen_cart;
  eigen_cart << start_[0], start_[1], .5, 0, 0, start_[2];
  eigen_start = _Cart2T(eigen_cart);
  Eigen::Matrix4d eigen_goal;
  eigen_cart << goal_[0], goal_[1], .5, 0, 0, goal_[2];
  eigen_goal = _Cart2T(eigen_cart);
  // Populate the VehicleStates
  start_state_ = VehicleState(Sophus::SE3d(eigen_start), start_[3], 0);
  goal_state_ = VehicleState(Sophus::SE3d(eigen_goal), goal_[3], 0);
}

//////////////////

bool PathPlannerTest::InitMesh(){
  // We don't want to reinitialize if we have the same map, after all.
  bullet_heightmap* map = new bullet_heightmap(heightmap_data_);
  btVector3 dMin(DBL_MAX,DBL_MAX,DBL_MAX);
  btVector3 dMax(DBL_MIN,DBL_MIN,DBL_MIN);
  CarParameters::LoadFromFile(params_file_name_, m_VehicleParams);
  // Set up our mesh
  SceneGraph::GLHeightmap* new_map =
      new SceneGraph::GLHeightmap(heightmap_data_->x_data_,
                                  heightmap_data_->y_data_,
                                  heightmap_data_->z_data_,
                                  heightmap_data_->row_count_,
                                  heightmap_data_->col_count_);
  planner_gui_.Init(new_map);
  LOG(INFO) << "Init our mesh!";
  // Set up our car
  car_model_ = new BulletCarModel();
  btTransform localTrans;
  localTrans.setIdentity();
  localTrans.setOrigin(btVector3(0,0,0));
  // MAKE SURE YOU ADD ENOUGH FREAKIN' WORLDS TO THE CAR MODEL.
  // 11 should do it.
  car_model_->Init(heightmap_data_->row_count_,
                   heightmap_data_->col_count_,
                   heightmap_data_->x_data_,
                   heightmap_data_->y_data_,
                   heightmap_data_->z_data_,
                   localTrans, dMin, dMax, m_VehicleParams, 11);
  GroundStates();
  planner_gui_.AddWaypoint(start_state_);
  planner_gui_.AddWaypoint(goal_state_);
  // In case we suspect discrepancy between the car physics and rendering
  // (usually around the wheel axes)
  // m_GLDebugDrawer.Init(car_model_);
  // planner_gui_.AddGLObject(&m_GLDebugDrawer);
  planner_gui_.AddCar(m_VehicleParams[CarParameters::WheelBase],
                      m_VehicleParams[CarParameters::Width],
                      m_VehicleParams[CarParameters::Height],
                      m_VehicleParams[CarParameters::WheelRadius],
                      m_VehicleParams[CarParameters::WheelWidth]);
}

//////////////////////////////////////////////////

void PathPlannerTest::GroundStates(){
  // Ground the start point.
  Eigen::Vector3d dIntersect, normal;
  Sophus::SE3d pose = start_state_.m_dTwv;
  if(car_model_->RayCast(pose.translation(), GetBasisVector(pose,2)*10,
                         dIntersect, true, 0)){
    dIntersect(2) = dIntersect(2)+.15;
    start_state_.m_dTwv.translation() = dIntersect;
  }

  // Ground the goal point
  pose = goal_state_.m_dTwv;
  if(car_model_->RayCast(pose.translation(), GetBasisVector(pose,2)*30,
                         dIntersect, true, 0)){
    dIntersect(2) = dIntersect(2)+.15;
    goal_state_.m_dTwv.translation() = dIntersect;
  }
}

/////////////////////////////////////////////

//Finds the fastest path between two
void PathPlannerTest::SolveTrajectory(pb::PlannerPolicyMsg* policy){
  bool success = false;
  int count = 0;
  int max_count = 1000;
  ApplyVelocitesFunctor5d func(car_model_, Eigen::Vector3d::Zero(), NULL);
  func.SetNoDelay(true);
  MotionSample sample;
  car_model_->SetState(0, start_state_);
  GLWayPoint* a = &planner_gui_.GetWaypoint(0)->m_Waypoint;
  GLWayPoint* b = &planner_gui_.GetWaypoint(1)->m_Waypoint;
  VehicleState st(Sophus::SE3d(a->GetPose4x4_po()),a->GetVelocity(),0);
  VehicleState gl(Sophus::SE3d(b->GetPose4x4_po()),b->GetVelocity(),0);
  LocalProblem problem(&func, st, gl, 1.0/30.0);
  local_planner_.InitializeLocalProblem(problem, 0, NULL, eCostPoint);

  BezierBoundaryProblem* spline = problem.GetBezierProblem();
  Eigen::Vector6d spline_vec;
  for (int ii = 0; ii<6; ii++) {
    spline_vec<<spline->x_values_(ii), spline->y_values_(ii), 0, 0, 0, 0;
    planner_gui_.AddSplinePoints(spline_vec, ii);
  }
  for (int ii = 0; ii<200; ii++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    planner_gui_.Render();
  }
  while (!success && count < max_count) {
    success = local_planner_.Iterate(problem);
    local_planner_.SimulateTrajectory(sample,problem,0,false);
    for (int ii = 0; ii<6; ii++) {
      LOG(INFO) << spline->x_values_(ii);
      LOG(INFO) << spline->y_values_(ii);
      spline_vec<<spline->x_values_(ii), spline->y_values_(ii), 0, 0, 0, 0;
      planner_gui_.MoveSplinePoints(spline_vec, ii);
      planner_gui_.Render();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // Render what we see
    if (count % 5 == 0) {
      for (int ii=0; ii<sample.m_vStates.size(); ii++){
        car_model_->SetState(0, sample.m_vStates.at(ii));
        planner_gui_.SetCarState(0, sample.m_vStates.at(ii), true);
        planner_gui_.Render();
      }
    }
    count++;
  }
  for (int ii = 0; ii<200; ii++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    planner_gui_.Render();
  }
  // We should have a good plan now
  // We can easily grab the bezier_boundary_problem from here.
  // BezierBoundaryProblem* spline = problem.GetBezierProblem();

  VehicleState last_vehicle_state = sample.GetLastPose();
  for(unsigned int ii=0; ii<sample.m_vCommands.size(); ii++){
    ControlCommand Comm = sample.m_vCommands.at(ii);
    policy->add_force(Comm.m_dForce);
    policy->add_phi(Comm.m_dPhi);
    policy->add_time(Comm.m_dT);
  }
  // Print forces here to verify results with PathPlanner program
  if (count >= max_count) {
    LOG(INFO) << "We're done with our policy search!!";
    LOG(INFO) << "We hit the maximum number of iterations...";
  } else {
    LOG(INFO) << "We're done with our policy search!!";
  }
}

///////////////////////////////////////////////////////
// Interpolates commands given a set of spline parameters
void PathPlannerTest::SampleTrajectory(pb::PlannerPolicyMsg* policy,
                                       Eigen::VectorXd x_values,
                                       Eigen::VectorXd y_values) {
  bool success = false;
  int count = 0;
  int max_count = 1000;
  ApplyVelocitesFunctor5d func(car_model_, Eigen::Vector3d::Zero(), NULL);
  func.SetNoDelay(true);
  MotionSample sample;
  car_model_->SetState(0, start_state_);
  GLWayPoint* a = &planner_gui_.GetWaypoint(0)->m_Waypoint;
  GLWayPoint* b = &planner_gui_.GetWaypoint(1)->m_Waypoint;
  VehicleState st(Sophus::SE3d(a->GetPose4x4_po()),a->GetVelocity(),0);
  VehicleState gl(Sophus::SE3d(b->GetPose4x4_po()),b->GetVelocity(),0);
  LocalProblem problem(&func, st, gl, 1.0/30.0);
  // Set our Bezier boundary problem's values
  // This means that it won't try to solve for the trajectory itself.
  // There's a nicer way to do this, I'm sure, but I haven't programmed it.
  BezierBoundaryProblem* spline = problem.GetBezierProblem();
  spline->x_values_ = x_values;
  spline->y_values_ = y_values;
  spline->has_given_points_ = true;
  Eigen::Vector6d spline_vec;
  for (int ii = 0; ii<6; ii++) {
    spline_vec<<spline->x_values_(ii), spline->y_values_(ii), 0, 0, 0, 0;
    planner_gui_.AddSplinePoints(spline_vec, ii);
  }
  local_planner_.InitializeLocalProblem(problem, 0, NULL, eCostPoint);
  for (int ii = 0; ii<200; ii++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    planner_gui_.Render();
  }
  // Solve our trajectory, and get the error
  local_planner_.Iterate(problem);
  local_planner_.SimulateTrajectory(sample, problem, 0, true);
  // Render what we see
  for (int jj = 0; jj < 5; jj++) {
    for (int ii=0; ii<sample.m_vStates.size(); ii++){
      car_model_->SetState(0, sample.m_vStates.at(ii));
      planner_gui_.SetCarState(0, sample.m_vStates.at(ii), true);
      planner_gui_.Render();
    }
  }
  for (int ii = 0; ii<200; ii++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    planner_gui_.Render();
  }

  VehicleState last_vehicle_state = sample.GetLastPose();
  for(unsigned int ii=0; ii<sample.m_vCommands.size(); ii++){
    ControlCommand Comm = sample.m_vCommands.at(ii);
    policy->add_force(Comm.m_dForce);
    policy->add_phi(Comm.m_dPhi);
    policy->add_time(Comm.m_dT);
  }
  // Print forces here to verify results with PathPlanner program
  if (count >= max_count) {
    LOG(INFO) << "We're done with our policy search!!";
    LOG(INFO) << "We hit the maximum number of iterations...";
  } else {
    LOG(INFO) << "We're done with our policy search!!";
  }
}

//////////////////

std::string PathPlannerTest::GetNumber(std::string name){
  std::size_t found = name.find("m");
  if(found!=std::string::npos){
    return name.substr(found+1);
  }
}
