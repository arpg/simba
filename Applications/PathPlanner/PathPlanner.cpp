#include "PathPlanner.h"

#include <thread>
#include <iostream>

PathPlanner::PathPlanner() {
  map_tau_ = 0; //Something random, why not.
  debug_level_ = 0;
  params_file_name_ =
      "/Users/Trystan/Code/simba/Applications/Examples/PathPlannerTest/gui_params.csv";
  ResetBooleans();
}

/////////////////////////////////////////////

/// DESTRUCTOR
PathPlanner::~PathPlanner() {

}

void PathPlanner::InitNode() {
  node_.init(planner_name_);
  // Provide RPC calls for all of our methods.
  node_.provide_rpc("SetConfiguration", &_SetConfiguration, this);
  node_.provide_rpc("SetHeightmap", &_SetHeightmap, this);
  node_.provide_rpc("GetStatus", &_GetStatus, this);
  node_.provide_rpc("GetPolicy", &_GetPolicy, this);
  node_.provide_rpc("GetMotionSample", &_GetMotionSample, this);
  node_.provide_rpc("GetSpline", &_GetSpline, this);
  ResetBooleans();
}

/////////////////////////////////////////////
/// SETTERS AND GETTERS FOR PATH PLANNER

void PathPlanner::SetConfiguration(pb::RegisterPlannerReqMsg& mRequest,
                                   pb::RegisterPlannerRepMsg& mReply) {
  // Get the configurations from the request
  master_node_name_ = mRequest.req_node_name();
  pb::PlannerConfigMsg config = mRequest.config();
  start_.resize(config.start_param().size());
  goal_.resize(config.goal_param().size());
  for (int ii = 0; ii < config.start_param().size(); ii++) {
    start_.at(ii) = config.start_param().Get(ii);
    goal_.at(ii) = config.goal_param().Get(ii);
  }
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
  mReply.set_success(1);
  config_set_ = true;
}

///////////

void PathPlanner::SetHeightmap(pb::RegisterPlannerReqMsg& mRequest,
                               pb::RegisterPlannerRepMsg& mReply) {
  // TODO : Change so that we grab parameters from mRequest
  pb::PlannerHeightmapMsg heightmap = mRequest.heightmap();
  // Create our world mesh
  int row_count = heightmap.row_count();
  int col_count = heightmap.col_count();
  std::vector<double> X, Y, Z;
  X.resize(row_count*col_count);
  Y.resize(row_count*col_count);
  Z.resize(row_count*col_count);
  for (int ii = 0; ii < heightmap.x_data().size(); ii++) {
    X.at(ii) = heightmap.x_data().Get(ii);
    Y.at(ii) = heightmap.y_data().Get(ii);
    Z.at(ii) = heightmap.z_data().Get(ii);
  }
  HeightmapShape* heightmap_data = new HeightmapShape("Map", row_count,
                                                      col_count,
                                                      X, Y, Z);
  bullet_heightmap* map = new bullet_heightmap(heightmap_data);
  btVector3 dMin(DBL_MAX,DBL_MAX,DBL_MAX);
  btVector3 dMax(DBL_MIN,DBL_MIN,DBL_MIN);
  CarParameters::LoadFromFile(params_file_name_, m_VehicleParams);
  LOG(debug_level_) << "Init our mesh!";
  // Set up our car
  car_model_ = new BulletCarModel();
  btTransform localTrans;
  localTrans.setIdentity();
  localTrans.setOrigin(btVector3(0,0,0));
  // MAKE SURE YOU ADD ENOUGH FREAKIN' WORLDS TO THE CAR MODEL.
  // 11 should do it.
  car_model_->Init(heightmap_data->row_count_,
                   heightmap_data->col_count_,
                   heightmap_data->x_data_,
                   heightmap_data->y_data_,
                   heightmap_data->z_data_,
                   localTrans, dMin, dMax, m_VehicleParams, 11);
  GroundStates();
  mReply.set_success(1);
  mesh_set_ = true;
}

///////////

void PathPlanner::GetStatus(pb::RegisterPlannerReqMsg& mRequest,
                            pb::RegisterPlannerRepMsg& mReply) {
  pb::PlannerStatusMsg* status = new pb::PlannerStatusMsg();
  status->set_config_set(config_set_);
  status->set_mesh_set(mesh_set_);
  status->set_policy_set(policy_set_);
  mReply.set_allocated_status(status);
  mReply.set_rep_node_name(planner_name_);
}

void PathPlanner::GetPolicy(pb::RegisterPlannerReqMsg& mRequest,
                            pb::RegisterPlannerRepMsg& mReply) {
  if (policy_set_) {
    pb::PlannerPolicyMsg* policy = new pb::PlannerPolicyMsg();
    for (unsigned int ii = 0; ii < trajectory_->m_vCommands.size(); ii++) {
      ControlCommand comm = trajectory_->m_vCommands.at(ii);
      policy->add_force(comm.m_dForce);
      policy->add_phi(comm.m_dPhi);
      policy->add_time(comm.m_dT);
    }
    mReply.set_allocated_policy(policy);
    mReply.set_rep_node_name(planner_name_);
  }
  // All of our booleans are true, so to reset, reset them first.
  // ResetBooleans();
}

void PathPlanner::GetMotionSample(pb::RegisterPlannerReqMsg& mRequest,
                                  pb::RegisterPlannerRepMsg& mReply) {
  if (policy_set_) {
    mReply.set_rep_node_name(planner_name_);
    //TODO
  }
}

void PathPlanner::GetSpline(pb::RegisterPlannerReqMsg& mRequest,
                            pb::RegisterPlannerRepMsg& mReply) {
  if (policy_set_) {
    pb::PlannerSplineMsg* spline_msg = new pb::PlannerSplineMsg();
    BezierBoundaryProblem* spline = local_problem_->GetBezierProblem();
    for (unsigned int ii = 0; ii < spline->x_values_.size(); ii++) {
      spline_msg->add_x_values(spline->x_values_(ii));
      spline_msg->add_y_values(spline->y_values_(ii));
    }
    for (unsigned int ii = 0; ii < 4; ii++) {
      spline_msg->add_solved_goal_pose(spline->solved_goal_pose_(ii));
    }
    mReply.set_allocated_spline(spline_msg);
    mReply.set_rep_node_name(planner_name_); // in case we implement a queue
  }
}

/////////////////////////////////////////////

void PathPlanner::GroundStates() {
  // Ground the start point.
  Eigen::Vector3d dIntersect, normal;
  Sophus::SE3d pose = start_state_.m_dTwv;
  if (car_model_->RayCast(pose.translation(), GetBasisVector(pose,2)*10,
                          dIntersect, true, 0)) {
    dIntersect(2) = dIntersect(2)+.12;
    start_state_.m_dTwv.translation() = dIntersect;
  }
  // Ground the goal point
  pose = goal_state_.m_dTwv;
  if (car_model_->RayCast(pose.translation(), GetBasisVector(pose,2)*30,
                          dIntersect, true, 0)) {
    dIntersect(2) = dIntersect(2)+.12;
    goal_state_.m_dTwv.translation() = dIntersect;
  }
}

/////////////////////////
// TODO: arguments here might have to be modified to accept different things.

//Finds the fastest path between two
void PathPlanner::SolveBVP() {
  bool success = false;
  int count = 0;
  ApplyVelocitesFunctor5d func(car_model_, Eigen::Vector3d::Zero(), NULL);
  VehicleState state;
  car_model_->GetVehicleState(0, state);
  if (state.IsAirborne()) {
    LOG(debug_level_) << "We are airborne. That's no good.";
  }
  func.SetNoDelay(true);
  MotionSample sample;
  //Initialize the problem
  LocalProblem problem(&func, start_state_, goal_state_, 1.0/30.0);
  local_planner_.InitializeLocalProblem(problem, 0, NULL, eCostPoint);
  while (!success && count<100) {
    // Problem: the car is in the air. Hmm.
    success = local_planner_.Iterate(problem);
    local_planner_.SimulateTrajectory(sample,problem,0,true);
    count++;
  }
  if (problem.is_inertial_control_active_) {
    local_planner_.CalculateTorqueCoefficients(problem,&sample);
    local_planner_.SimulateTrajectory(sample,problem,0,true);
  }
  LOG(debug_level_) << "SUCCESS: Planned this configuration";
  local_problem_ = &problem;
  trajectory_ = &sample;
  policy_set_ = true;
}

///////////////

std::string PathPlanner::GetNumber(std::string name) {
  std::size_t found = name.find("m");
  if (found!=std::string::npos) {
    return name.substr(found+1);
  }
}

void PathPlanner::ResetBooleans() {
  config_set_ = false;
  mesh_set_ = false;
  policy_set_ = false;
  policy_delivered_ = false;
}

/***********************************
 * THE MAIN LOOP
 * Initializes PathPlanner, Runs the optimizer, and returns the path.
 * Never stops (though it does have a sleep time); instead, if it's done with
 * one path, it destructs the PathPlanner and creates a new one with the
 * new info it's fed.
 **********************************/

int main(int argc, char** argv) {
  PathPlanner* planner = new PathPlanner();
  std::string name = argv[1];
  planner->planner_name_ = name;
  std::string number = planner->GetNumber(planner->planner_name_);
  planner->InitNode();
  //We're planning forever, so never break this cycle
  while (!planner->config_set_ || !planner->mesh_set_) {
    // Just wait
    // Once these are set, we're ready to solve
  }
  planner->SolveBVP();
  while (!planner->policy_set_) {
    // Just wait again
  }
  while (planner->policy_set_
         || planner->config_set_
         || planner->mesh_set_) {
    // Just wait again for a reset
  }
}
