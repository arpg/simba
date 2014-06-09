#include "PathPlanner.h"

#include <thread>

PathPlanner::PathPlanner(){
  m_nTau = 8; //Something random, why not.
  params_file_name_ =
      "/Users/Trystan/Code/simba/Applications/PathPlanner/raycast_params.csv";
  need_BVP_ = true;
  solved_BVP_ = false;
}

/////////////////////////////////////////////

/// DESTRUCTOR
PathPlanner::~PathPlanner(){

}

void PathPlanner::Init(){
  m_Node.init(sim_planner_name_);
  cout<<"-------------------"<<GetNumber(sim_planner_name_)<<endl;
  while(!m_Node.subscribe("MATLAB/BVP"+GetNumber(sim_planner_name_))){
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    cout<<"->";
  }
  m_Node.advertise("CheckNeed");
  m_Node.advertise("CheckSolved");
  m_Node.advertise("Policy");
}

/////////////////////////////////////////////

pb::BVP_policy PathPlanner::StartPolicy(pb::BVP_params params){
  InitMesh(params);
  InitGoals(params);
  pb::BVP_policy policy = SampleTrajectory();
  return policy;
}

/////////////////////////////////////////////

///INITIALIZERS
bool PathPlanner::InitMesh(pb::BVP_params params){
  // We don't want to reinitialize if we have the same map, after all.
  if(m_nTau!=params.tau()){
    delete car_model_;
    car_model_ = new BulletCarModel;
    // Can I do this?
    m_nTau = params.tau();
    // Create our world mesh
    int row_count = params.row_count();
    int col_count = params.col_count();
    std::vector<double> X, Y, Z;
    X.resize(row_count*col_count);
    Y.resize(row_count*col_count);
    Z.resize(row_count*col_count);
    for (int ii=0; ii < params.x_data().size(); ii++) {
      X.at(ii) = params.x_data().Get(ii);
      Y.at(ii) = params.y_data().Get(ii);
      Z.at(ii) = params.z_data().Get(ii);
    }
    HeightmapShape* MapShape = new HeightmapShape("Map", row_count, col_count,
                                                  X, Y, Z);
    bullet_heightmap* map = new bullet_heightmap(MapShape);
    btVector3 dMin(DBL_MAX,DBL_MAX,DBL_MAX);
    btVector3 dMax(DBL_MIN,DBL_MIN,DBL_MIN);
    CarParameters::LoadFromFile(params_file_name_, m_VehicleParams);
    // MAKE SURE YOU ADD ENOUGH FREAKIN' WORLDS TO THE CAR MODEL.
    // 11 should do it.
    car_model_->Init(map->getBulletShapePtr(), dMin, dMax, m_VehicleParams, 11);
    return true;
  }
  return false;
}

/////////////////////////////////////////////

double* PathPlanner::RaycastToGround(){

  // TODO: Fix this representation.

  Sophus::SE3d Twv = start_state_.m_dTwv;
  double x = Twv.translation()(0);
  double y = Twv.translation()(1);
  BulletWorldInstance* world = car_model_->GetWorldInstance(0);
  RaycastVehicle* Vehicle = world->m_pVehicle;

  // Move our vehicle out of the way...
  btVector3 point(x+50, y+50, -100);
  btMatrix3x3 rot = Vehicle->getChassisWorldTransform().getBasis();
  btTransform bullet_trans(rot, point);
  Vehicle->getRigidBody()->setCenterOfMassTransform(bullet_trans);

  // Now shoot our ray...
  btVector3 ray_start(x, y, 100);
  btVector3 ray_end(x, y, -100);
  btCollisionWorld::ClosestRayResultCallback ray_callback(ray_start,
                                                          ray_end);
  world->m_pDynamicsWorld->rayTest(ray_start, ray_end, ray_callback);
  btVector3 hitpoint = Vehicle->getChassisWorldTransform().getOrigin();
  if(ray_callback.hasHit()){
    hitpoint = ray_callback.m_hitPointWorld;
    WheelInfo wheel = Vehicle->getWheelInfo(2);
    double radius = wheel.m_wheelsRadius;
    hitpoint.setZ(hitpoint.getZ() +
                  m_VehicleParams.at(CarParameters::SuspConnectionHeight) +
                  m_VehicleParams.at(CarParameters::WheelRadius) +
                  0.5); // Just for good measure.
  }

  // Now move our car!
  btTransform bullet_move(rot, hitpoint);
  Vehicle->getRigidBody()->setCenterOfMassTransform(bullet_move);

  //Now make sure none of our wheels are in the ground.
  //Kind of a nasty oop, but keep it for now.
  double hit = -1;
  double count = 0;
  while(hit==-1 && count<20){
    for(int i = 0; i<4; i++){
      hit = Vehicle->rayCast(Vehicle->getWheelInfo(i));
      if(hit!=-1){
        break;
      }
    }
    //If we're still in the ground, lift us up!
    hitpoint.setZ(hitpoint.getZ()+.1);
    btTransform bullet_move(rot, hitpoint);
    Vehicle->getRigidBody()->setCenterOfMassTransform(bullet_move);
    if(hit!=-1){
      break;
    }
    count++;
    if(count==20){
      break;
    }
  }
  int on = false;
  btVector3 VehiclePose = Vehicle->getChassisWorldTransform().getOrigin();
  double dT = 1.0/30.0;
  while(on == 0){
    on = OnTheGround(Vehicle);
    world->m_pDynamicsWorld->stepSimulation(dT,1,dT);
    VehiclePose = Vehicle->getChassisWorldTransform().getOrigin();
  }

  start_state_.m_dTwv.translation()<<VehiclePose.getX(),
      VehiclePose.getY(), VehiclePose.getZ();
  // return pose
  ;

}

// This just drops us off on the surface...
int PathPlanner::OnTheGround(RaycastVehicle* vehicle){
  int OnGround = 0;
  int hit = 0;
  for(int i = 0; i<4; i++){
    hit = hit + vehicle->rayCast(vehicle->getWheelInfo(i));
  }
  if(hit==0){
    OnGround = 1;
  }
  return OnGround;
}

///

//void SetVehicleVels(double* lin_vel, double* ang_vel){
//  BulletWorldInstance* world = car_model_->GetWorldInstance(0);
//  RaycastVehicle* Vehicle = world->m_pVehicle;
//  RigidBodyPtr VehicleBody = ->m_pRigidBody;
//  VehiclePtr Vehicle = m_mRayVehicles[id]->m_pVehicle;
//  btVector3 Lin(lin_vel[0], lin_vel[1], lin_vel[2]);
//  btVector3 Ang(ang_vel[0], ang_vel[1], ang_vel[2]);
//  VehicleBody->setLinearVelocity(Lin);
//  VehicleBody->setAngularVelocity(Ang);
//  Vehicle->resetSuspension();
//}
//
//void ResetVehicle(double id, double* start_pose, double* start_rot){
//  BulletWorldInstance* world = car_model_->GetWorldInstance(0);
//  RaycastVehicle* Vehicle = world->m_pVehicle;
//  // Move the vehicle into start position
//  btMatrix3x3 rot(start_rot[0], start_rot[3], start_rot[6],
//                  start_rot[1], start_rot[4], start_rot[7],
//                  start_rot[2], start_rot[5], start_rot[8]);
//  btVector3 pose(start_pose[0], start_pose[1], start_pose[2]);
//  btTransform bullet_trans(rot, pose);
//  // Reset our car to its initial state.
//  m_mRayVehicles[id]->m_pRigidBody->setCenterOfMassTransform(bullet_trans);
//}

/////////////////////////////////////////////

void PathPlanner::GroundStates(){
  // Ground the start point.
  Eigen::Vector3d dIntersect;
  Eigen::Vector3d normal;
  Sophus::SE3d pose = start_state_.m_dTwv;
  if(car_model_->RayCast(pose.translation(), GetBasisVector(pose,2)*10,
                         dIntersect, true, 0)){
    pose.translation() = dIntersect;
    if(car_model_->RayCastNormal(pose.translation(),
                                 GetBasisVector(start_state_.m_dTwv,2),
                                 normal, 0)){
      normal = normal.normalized();
      Eigen::Quaternion<double> quatRot(Eigen::AngleAxis<double>(
          start_state_.GetTheta(), normal));
      Eigen::Matrix3d rotMat = quatRot*start_state_.m_dTwv.rotationMatrix();
      start_state_.m_dTwv =
          Sophus::SE3d(rotMat, start_state_.m_dTwv.translation());
      cout<<"Setting start point to ground..."<<endl;
      cout<<start_state_.m_dTwv.matrix()<<endl;
    }

    // Ground the goal point
    pose = goal_state_.m_dTwv;
    if(car_model_->RayCast(pose.translation(), GetBasisVector(pose,2)*30,
                           dIntersect, true, 0)){
      pose.translation() = dIntersect;
      if(car_model_->RayCastNormal(pose.translation(),
                                   GetBasisVector(pose,2)*10,
                                   dIntersect, 0)){
        normal = normal.normalized();
        Eigen::Quaternion<double> quatRot(Eigen::AngleAxis<double>(
            goal_state_.GetTheta(), normal));
        Eigen::Matrix3d rotMat = quatRot*goal_state_.m_dTwv.rotationMatrix();
        goal_state_.m_dTwv =
            Sophus::SE3d(rotMat, goal_state_.m_dTwv.translation());
        cout<<"Setting goal point to ground..."<<endl;
        cout<<goal_state_.m_dTwv.matrix()<<endl;
      }
    }
  }
}

//////////////////

// TODO : get GroundStates replaced with RaycastToGround process.

void PathPlanner::InitGoals(pb::BVP_params params){
  // Now populate the start and goal parameters.
  // X, Y, yaw, and velocity... that should be it.
  const double* start = params.start_param().data();
  const double* goal = params.goal_param().data();
  for(int ii = 0; ii<4; ii++){
    start_.push_back(start[ii]);
    goal_.push_back(goal[ii]);
  }
  Eigen::Matrix4d eigen_start;
  Eigen::Vector6d eigen_cart;
  eigen_cart<<start[0], start[1], 0, 0, 0, start[2];
  eigen_start = _Cart2T(eigen_cart);
  Eigen::Matrix4d eigen_goal;
  eigen_cart<<goal[0], goal[1], 0, 0, 0, goal[2];
  eigen_goal = _Cart2T(eigen_cart);
  // Populate the VehicleStates
  start_state_ = VehicleState(Sophus::SE3d(eigen_start), start[3], 0);
  goal_state_ = VehicleState(Sophus::SE3d(eigen_goal), goal[3], 0);
  RaycastToGround();
  if (start_state_.IsAirborne()) {
    std::cout<<"Whaaaaaaaaaaat; we are airborne. That's no good.";
  }
  car_model_->SetState(0, start_state_);
}

/////////////////////////////////////////////

//Finds the fastest path between two
pb::BVP_policy PathPlanner::SampleTrajectory(){
  bool success = false;
  int count = 0;
  ApplyVelocitesFunctor5d func(car_model_, Eigen::Vector3d::Zero(), NULL);
  VehicleState state;
  car_model_->GetVehicleState(0, state);
  if (state.IsAirborne()) {
    std::cout<<"We are airborne. That's no good.";
  }
  func.SetNoDelay(true);
  MotionSample sample;
  //Initialize the problem
  LocalProblem problem(&func, start_state_, goal_state_, 1.0/30.0);
  m_snapper.InitializeLocalProblem(problem, 0, NULL, eCostPoint);
  while(!success && count<100){
    // Problem: the car is in the air. Hmm.
    success = m_snapper.Iterate(problem);
    m_snapper.SimulateTrajectory(sample,problem,0,true);
    count++;
  }
  if(problem.m_bInertialControlActive){
    m_snapper.CalculateTorqueCoefficients(problem,&sample);
    m_snapper.SimulateTrajectory(sample,problem,0,true);
  }
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
  cout<<"We're done with our policy search!!"<<endl;
  policy.set_tau(m_nTau);
  return policy;
}

std::string PathPlanner::GetNumber(std::string name){
  std::size_t found = name.find("m");
  if(found!=std::string::npos){
    return name.substr(found+1);
  }
}

/***********************************
 * THE MAIN LOOP
 * Initializes PathPlanner, Runs the optimizer, and returns the path.
 * Never stops (though it does have a sleep time); instead, if it's done with
 * one path, it destructs the PathPlanner and creates a new one with the
 * new info it's fed.
 **********************************/

int main(int argc, char** argv){
  PathPlanner* sim = new PathPlanner();
  std::string name = argv[1];
  sim->sim_planner_name_ = name;
  std::string number = sim->GetNumber(sim->sim_planner_name_);
  cout<<"Sim planner name is "<<sim->sim_planner_name_<<endl;
  sim->Init();
  while(1){
    int count = 0;
    pb::BVP_check ineed_bvp;
    ineed_bvp.set_need(true);
    while(!sim->m_Node.publish("CheckNeed", ineed_bvp)){
      ineed_bvp.set_need(true);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    pb::BVP_params params;
    std::cout<<"Starting to receive parameters..."<<std::endl;
    while(!sim->m_Node.receive("MATLAB/BVP"+number, params) && count<100){
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      count++;
    }
    pb::BVP_policy policy;
    if (count<100) {
      policy = sim->StartPolicy(params);
      pb::BVP_check bvp_solved;
      bvp_solved.set_need(true);
      while (!sim->m_Node.publish("CheckSolved", bvp_solved) && count<10){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      while(!sim->m_Node.publish("Policy", policy) && count<10){
        std::cout<<"Sending policy..."<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout<<"Ready for the next plan!!"<<std::endl;
      }
    }
    std::cout<<"False alarm; no plan here."<<std::endl;
  }
}
