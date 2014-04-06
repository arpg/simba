#include "SimPlanner.h"

#include <thread>

SimPlanner::SimPlanner(){
  m_nTau = 8; //Something random, why not.
  need_BVP_ = true;
  solved_BVP_ = false;
}

/////////////////////////////////////////////

/// DESTRUCTOR
SimPlanner::~SimPlanner(){

}

void SimPlanner::Init(){
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

pb::BVP_policy SimPlanner::StartPolicy(pb::BVP_params params){
  InitMesh(params);
  InitGoals(params);
  pb::BVP_policy policy = SampleTrajectory();
  return policy;
}

/////////////////////////////////////////////

///INITIALIZERS
bool SimPlanner::InitMesh(pb::BVP_params params){
  // We don't want to reinitialize if we have the same map, after all.
  if(m_nTau!=params.tau()){
    delete m_CarModel;
    m_CarModel = new BulletCarModel;
    // Can I do this?
    m_nTau = params.tau();
    // Create our world mesh
    int row_count = params.row_count();
    int col_count = params.col_count();

    // I think this should work?
    double* X = new double[row_count*col_count];
    double* Y = new double[row_count*col_count];
    double* Z = new double[row_count*col_count];
    for (int ii=0; ii < params.x_data().size(); ii++) {
      X[ii] = params.x_data().Get(ii);
      Y[ii] = params.y_data().Get(ii);
      Z[ii] = params.z_data().Get(ii);
    }
    HeightmapShape* MapShape = new HeightmapShape("Map", row_count, col_count,
                                                  X, Y, Z);
    bullet_heightmap* map = new bullet_heightmap(MapShape);
    btVector3 dMin(DBL_MAX,DBL_MAX,DBL_MAX);
    btVector3 dMax(DBL_MIN,DBL_MIN,DBL_MIN);
    CarParameters::LoadFromFile(PARAMS_FILE_NAME,m_VehicleParams);
    // MAKE SURE YOU ADD ENOUGH FREAKIN' WORLDS.
    // 11 should do it.
    m_CarModel->Init(map->getBulletShapePtr(), dMin, dMax, m_VehicleParams, 11);
    return true;
  }
  return false;
}

/////////////////////////////////////////////

void SimPlanner::GroundStates(){
  cout<<"GSOO"<<endl;
  // Ground the start point.
  Eigen::Vector3d dIntersect;
  Eigen::Vector3d normal;
  Sophus::SE3d pose = m_vsStart.m_dTwv;
  if(m_CarModel->RayCast(pose.translation(), GetBasisVector(pose,2)*10,
                         dIntersect, true, 0)){
    pose.translation() = dIntersect;
    if(m_CarModel->RayCastNormal(pose.translation(),
                                 GetBasisVector(m_vsStart.m_dTwv,2),
                                 normal, 0)){
      normal = normal.normalized();
      Eigen::Quaternion<double> quatRot(Eigen::AngleAxis<double>(
                                          m_vsStart.GetTheta(), normal));
      Eigen::Matrix3d rotMat = quatRot*m_vsStart.m_dTwv.rotationMatrix();
      m_vsStart.m_dTwv = Sophus::SE3d(rotMat, m_vsStart.m_dTwv.translation());
      cout<<"GSOO"<<endl;
      cout<<m_vsStart.m_dTwv.matrix()<<endl;
    }

    // Ground the goal point
    pose = m_vsGoal.m_dTwv;
    if(m_CarModel->RayCast(pose.translation(), GetBasisVector(pose,2)*10,
                           dIntersect, true, 0)){
      pose.translation() = dIntersect;
      if(m_CarModel->RayCastNormal(pose.translation(),
                                   GetBasisVector(pose,2)*10,
                                   dIntersect, 0)){
        normal = normal.normalized();
        Eigen::Quaternion<double> quatRot(Eigen::AngleAxis<double>(
                                            m_vsGoal.GetTheta(), normal));
        Eigen::Matrix3d rotMat = quatRot*m_vsGoal.m_dTwv.rotationMatrix();
        m_vsGoal.m_dTwv = Sophus::SE3d(rotMat, m_vsGoal.m_dTwv.translation());
        cout<<"GSOO"<<endl;
        cout<<m_vsGoal.m_dTwv.matrix()<<endl;
      }
    }
  }
}

//////////////////

void SimPlanner::InitGoals(pb::BVP_params params){
  // Now populate the start and goal parameters.
  // X, Y, yaw, and velocity... that should be it.
  const double* start = params.start_param().data();
  const double* goal = params.goal_param().data();
  Eigen::Matrix4d eigen_start;
  Eigen::Vector6d eigen_cart;
  eigen_cart<<start[0], start[1], 0, 0, 0, start[2];
  eigen_start = _Cart2T(eigen_cart);
  Eigen::Matrix4d eigen_goal;
  eigen_cart<<goal[0], goal[1], 0, 0, 0, goal[2];
  eigen_goal = _Cart2T(eigen_cart);
  // Populate the VehicleStates
  m_vsStart = VehicleState(Sophus::SE3d(eigen_start), start[3], 0);
  m_vsGoal = VehicleState(Sophus::SE3d(eigen_goal), goal[3], 0);
  GroundStates();
}

/////////////////////////////////////////////

//Finds the fastest path between two
pb::BVP_policy SimPlanner::SampleTrajectory(){
  bool success = false;
  int count = 0;
  ApplyVelocitesFunctor5d func(m_CarModel, Eigen::Vector3d::Zero(), NULL);
  func.SetNoDelay(true);
  MotionSample sample;
  //Initialize the problem
  cout<<m_vsStart.m_dTwv.matrix()<<endl;
  cout<<m_vsGoal.m_dTwv.matrix()<<endl;
  LocalProblem problem(&func, m_vsStart, m_vsGoal, 1.0/30.0);
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
  for(unsigned int ii=0; ii<sample.m_vCommands.size(); ii++){
    ControlCommand Comm = sample.m_vCommands.at(ii);
    policy.add_force(Comm.m_dForce);
    policy.add_phi(Comm.m_dPhi);
    policy.add_time(Comm.m_dT);
  }
  cout<<"STOO"<<endl;
  policy.set_tau(m_nTau);
  return policy;
}

std::string SimPlanner::GetNumber(std::string name){
  std::size_t found = name.find("m");
  if(found!=std::string::npos){
    return name.substr(found+1);
  }
}

/***********************************
 * THE MAIN LOOP
 * Initializes SimPlanner, Runs the optimizer, and returns the path.
 * Never stops (though it does have a sleep time); instead, if it's done with
 * one path, it destructs the SimPlanner and creates a new one with the
 * new info it's fed.
 **********************************/

int main(int argc, char** argv){
  SimPlanner* sim = new SimPlanner();
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
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    pb::BVP_params params;
    std::cout<<"Starting to receive parameters..."<<std::endl;
    while(!sim->m_Node.receive("MATLAB/BVP"+number, params) && count<5){
      std::this_thread::sleep_for(std::chrono::seconds(1));
      count++;
    }
    pb::BVP_policy policy;
    if (count<5) {
      policy = sim->StartPolicy(params);
    }
    pb::BVP_check bvp_solved;
    bvp_solved.set_need(true);
    while (!sim->m_Node.publish("CheckSolved", bvp_solved) && count<5){
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    while(!sim->m_Node.publish("Policy", policy) && count<10){
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}
