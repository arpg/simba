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

void PathPlannerTest::Init(HeightmapShape* heightmap_data){
  heightmap_data_ = heightmap_data;
  InitGoals();
  InitMesh();
}

/////////////////////////////////////////////

void PathPlannerTest::InitGoals(){
  // <x, y, theta, vel>
  start_.push_back(-.5);
  start_.push_back(1);
  start_.push_back(0);
  start_.push_back(1);
  goal_.push_back(1.8);
  goal_.push_back(1.5);
  goal_.push_back(.5);
  goal_.push_back(1);
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
  LOG(INFO) << start_state_.m_dTwv.matrix();
  LOG(INFO) << goal_state_.m_dTwv.matrix();
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
    dIntersect(2) = dIntersect(2)+.12;
    start_state_.m_dTwv.translation() = dIntersect;
    // LOG(INFO) << pose.translation();
    // if(car_model_->RayCastNormal(pose.translation(),
    //                              GetBasisVector(start_state_.m_dTwv,2),
    //                              normal, 0)){
    //   Eigen::Quaternion<double> quatRot(Eigen::AngleAxis<double>(
    //       start_state_.GetTheta(), normal));
    //   Eigen::Matrix3d rotMat = quatRot*start_state_.m_dTwv.rotationMatrix();
    //   start_state_.m_dTwv =
    //       Sophus::SE3d(rotMat, start_state_.m_dTwv.translation());
    //   LOG(INFO) << "Setting start point to ground...";
    }

    // Ground the goal point
    pose = goal_state_.m_dTwv;
    if(car_model_->RayCast(pose.translation(), GetBasisVector(pose,2)*30,
                           dIntersect, true, 0)){
      dIntersect(2) = dIntersect(2)+.12;
      goal_state_.m_dTwv.translation() = dIntersect;
      // if(car_model_->RayCastNormal(pose.translation(),
      //                              GetBasisVector(pose,2)*10,
      //                              dIntersect, 0)){
      //   normal = normal.normalized();
      //   Eigen::Quaternion<double> quatRot(Eigen::AngleAxis<double>(
      //       goal_state_.GetTheta(), normal));
      //   Eigen::Matrix3d rotMat = quatRot*goal_state_.m_dTwv.rotationMatrix();
      //   goal_state_.m_dTwv =
      //       Sophus::SE3d(rotMat, goal_state_.m_dTwv.translation());
      //   LOG(INFO) << "Setting goal point to ground...";
      }
    }
//   }
// }

/////////////////////////////////////////////

//Finds the fastest path between two
void PathPlannerTest::SampleTrajectory(pb::PlannerPolicyMsg* policy){
  bool success = false;
  int count = 0;
  ApplyVelocitesFunctor5d func(car_model_, Eigen::Vector3d::Zero(), NULL);
  func.SetNoDelay(true);
  MotionSample sample;
  car_model_->SetState(0, start_state_);
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
      car_model_->SetState(0, sample.m_vStates.at(ii));
      planner_gui_.SetCarState(0, sample.m_vStates.at(ii), true);
      LOG(INFO) << std::endl
                << sample.m_vStates.at(ii).m_dTwv.matrix();
      planner_gui_.Render();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    count++;
  }
  if (problem.is_inertial_control_active_) {
    m_snapper.CalculateTorqueCoefficients(problem, &sample);
    m_snapper.SimulateTrajectory(sample,problem,0,true);
  }
  // We should have a good plan now
  // We can easily grab the bezier_boundary_problem from here.
  BezierBoundaryProblem* spline = problem.GetBezierProblem();

  VehicleState last_vehicle_state = sample.GetLastPose();
  for(unsigned int ii=0; ii<sample.m_vCommands.size(); ii++){
    ControlCommand Comm = sample.m_vCommands.at(ii);
    policy->add_force(Comm.m_dForce);
    policy->add_phi(Comm.m_dPhi);
    policy->add_time(Comm.m_dT);
  }
  // Print forces here to verify results with PathPlanner program
  LOG(INFO) << "X values: ";
  LOG(INFO) << std::endl << spline->x_values_;
  LOG(INFO) << "Y vaules: ";
  LOG(INFO) << std::endl << spline->y_values_;
  if(count==100){
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

/***********************************
 * THE MAIN LOOP
 * This is a test, so we're only going to do ONE path through
 * the simplest type of a terrain, the flat plane. Get the commands
 * back from this simulation, and start a separate Node process
 * that sends these commands to LocalSim in SimBA.
 **********************************/

int main(int argc, char** argv){

  // Put a PlannerGui here, so that we can see what we're doing.

  PathPlannerTest* sim = new PathPlannerTest();
  std::string name = "Sim0";
  if (argc == 2) {
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
  pb::PlannerPolicyMsg* policy = new pb::PlannerPolicyMsg();
  sim->SampleTrajectory(policy);
  // 3. Start looking for the car (similar to KeyboardCommander)
  KeyCarController KeyCar("NodeCar:[name=VehicleController,sim=Ricky]//",
                          policy);
  KeyCar.ApplyCommands();
  LOG(INFO) << "Done with all of our shenanigans!";
}
