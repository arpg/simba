#include "PlannerLibTest.h"

#include <thread>

PlannerLibTest::PlannerLibTest(){
  params_file_name_ =
      "/Users/Trystan/Code/simba/Applications/SimPlanner/raycast_params.csv";
}

/////////////////////////////////////////////

/// DESTRUCTOR
PlannerLibTest::~PlannerLibTest(){
}

/////////////////////////////////////////////

void PlannerLibTest::Init(HeightmapShape* heightmap_data){
  heightmap_data_ = heightmap_data;
  // <x, y, theta, vel>
  start_.push_back(10);
  start_.push_back(-5);
  start_.push_back(0);
  start_.push_back(0);
  goal_.push_back(10);
  goal_.push_back(15);
  goal_.push_back(0);
  goal_.push_back(1);
}

/////////////////////////////////////////////
// We've replaced what would be a call to a PbMsgs with
// a call to the heightmap_data_ member variable.
/////////TODO: Finish editing InitMesh, InitGoals, and SampleTrajectory
pb::BVP_policy PlannerLibTest::StartPolicy(){
  InitMesh();
  InitGoals();
  pb::BVP_policy policy = SampleTrajectory();
  return policy;
}

/////////////////////////////////////////////

///INITIALIZERS
bool PlannerLibTest::InitMesh(){
  // We don't want to reinitialize if we have the same map, after all.
  bullet_heightmap* map = new bullet_heightmap(heightmap_data_);
  btVector3 dMin(DBL_MAX,DBL_MAX,DBL_MAX);
  btVector3 dMax(DBL_MIN,DBL_MIN,DBL_MIN);
  CarParameters::LoadFromFile(params_file_name_, m_VehicleParams);
  // MAKE SURE YOU ADD ENOUGH FREAKIN' WORLDS TO THE CAR MODEL.
  // 11 should do it.
  car_model_ = new BulletCarModel();
  car_model_->Init(map->getBulletShapePtr(), dMin, dMax, m_VehicleParams, 11);
}

/////////////////////////////////////////////

double* PlannerLibTest::RaycastToGround(){
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

///////////////////////////////////

// This just drops us off on the surface...
int PlannerLibTest::OnTheGround(RaycastVehicle* vehicle){
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

void PlannerLibTest::GroundStates(){
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
void PlannerLibTest::InitGoals(){
  // Now populate the start and goal parameters.
  // X, Y, yaw, and velocity... that should be it.
  Eigen::Matrix4d eigen_start;
  Eigen::Vector6d eigen_cart;
  eigen_cart<<start_[0], start_[1], 0, 0, 0, start_[2];
  eigen_start = _Cart2T(eigen_cart);
  Eigen::Matrix4d eigen_goal;
  eigen_cart<<goal_[0], goal_[1], 0, 0, 0, goal_[2];
  eigen_goal = _Cart2T(eigen_cart);
  // Populate the VehicleStates
  start_state_ = VehicleState(Sophus::SE3d(eigen_start), start_[3], 0);
  goal_state_ = VehicleState(Sophus::SE3d(eigen_goal), goal_[3], 0);
  // TODO: Make sure that we get RaycastToGround working instead of
  // GroundStates
  // GroundStates();
  RaycastToGround();
  if (start_state_.IsAirborne()) {
    std::cout<<"Whaaaaaaaaaaat; we are airborne. That's no good.";
  }
  car_model_->SetState(0, start_state_);
}

/////////////////////////////////////////////

//Finds the fastest path between two
pb::BVP_policy PlannerLibTest::SampleTrajectory(){
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
  ////// TODO
  // We're going to import all of the mesh data from a .xml
  // instead of going through MATLAB, just to show proof
  // of concept. Look in URDF_Parser.cpp for this behavior; the data should
  // all be kept in Robot.xml
  // Once we do this, we should be able to get rid of most of the Node
  // messiness up until this program connects with LocalSim.
  PlannerLibTest* sim = new PlannerLibTest();
  std::string name = argv[1];
  sim->sim_planner_name_ = name;
  std::string number = sim->GetNumber(sim->sim_planner_name_);
  URDF_Parser* parser = new URDF_Parser();
  // 1. Read URDF files.
  XMLDocument WorldURDF;
  const string& sWorldURDFPath =
      "/Users/Trystan/Code/simba/urdf/HeightmapWorld.xml";
  GetXMLdoc(sWorldURDFPath, WorldURDF);
  sim->heightmap_data_ = parser->GetMeshData(WorldURDF);
  sim->Init(sim->heightmap_data_);
  int count = 0;
  // 2. Solve for the path using PlannerLib
  pb::BVP_policy policy;
  if (count<100) {
    policy = sim->StartPolicy();
  }
  // This is our solved policy, which we will send through this
  // interwebs.
  // 3. Start looking for the car (similar to KeyboardCommander)
  KeyCarController KeyCar("NodeCar:[name=VehicleController]//",
                          policy);
  while(1){
    // 4. Drive our car to the EDGE OF SPACE
    KeyCar.ApplyCommands();
    //  Send the commands to the car
    // These are the parts of policy:
    //   policy.add_force(Comm.m_dForce);
    //   policy.add_phi(Comm.m_dPhi);
    //   policy.add_time(Comm.m_dT);
    // for(unsigned int jj=0; jj<4; jj++){
    //   policy.add_start_param(start_.at(jj));
    //   policy.add_goal_param(goal_.at(jj));
    // }
    // policy.set_tau(m_nTau);
  }
  std::cout<<"Done with all of our shenanigans!"<<std::endl;
}
