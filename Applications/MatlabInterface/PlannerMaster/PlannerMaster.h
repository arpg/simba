#ifndef _PLANNERMASTER_H
#define _PLANNERMASTER_H

/*
 * File:   PlannerMaster.h
 * Author: bminortx
 * This wrapper is designed to facilitate the use of Node, in otder to pass
 * commands to PathPlanner
 */

#include <unistd.h>
#include <cstdlib>
#include "Node.h"

// For saving our code
#include <mat.h>
// Our protobuf sources - I couldn't find a successful way to import these
// using the makefile, so #includes had to do.
#include <BVP.pb.h>
#include "/Users/Trystan/Code/simba/build/SimBA/PB_Headers/BVP.pb.cc"

class PlannerMaster{
 public:

  /// CONSTRUCTOR
  PlannerMaster() {
    node_name_ = "PlannerMaster";
    node_.init(node_name_);
  }

  /// SETTERS AND GETTERS

  int SetConfiguration(int planner_num, double* start_point,
                       double* goal_point) {
    pb::RegisterPlannerReqMsg req;
    pb::RegisterPlannerRepMsg rep;
    pb::PlannerConfigMsg* config = new pb::PlannerConfigMsg();
    for (int ii = 0; ii < 4; ii++) {
      config->add_start_param(start_point[ii]);
      config->add_goal_param(goal_point[ii]);
    }
    req.set_req_node_name(node_name_);
    req.set_allocated_config(config);
    bool good = node_.call_rpc("Sim" + std::to_string(planner_num)
                               + "/SetConfiguration", req, rep);
    return rep.success();
  }

  int SetHeightmap(int planner_num, double* x_data, double* y_data,
                   double* z_data, int row_count, int col_count) {
    pb::RegisterPlannerReqMsg req;
    pb::RegisterPlannerRepMsg rep;
    pb::PlannerHeightmapMsg* map = new pb::PlannerHeightmapMsg();
    for (int ii = 0; ii < (row_count * col_count); ii++) {
      map->add_x_data(x_data[ii]);
      map->add_y_data(y_data[ii]);
      map->add_z_data(z_data[ii]);
    }
    map->set_col_count(col_count);
    map->set_row_count(row_count);
    req.set_req_node_name(node_name_);
    req.set_allocated_heightmap(map);
    node_.call_rpc("Sim" + std::to_string(planner_num)
                   +"/SetHeightmap", req, rep);
    return rep.success();

  }

  double* GetStatus(int planner_num) {
    pb::RegisterPlannerReqMsg req;
    pb::RegisterPlannerRepMsg rep;
    // give and take should never both be true.
    double* status = new double[3];
    status[0] = 0;
    status[1] = 0;
    status[2] = 0;
    req.set_req_node_name(node_name_);
    node_.call_rpc("Sim" + std::to_string(planner_num)
                   + "/GetStatus", req, rep);
    pb::PlannerStatusMsg status_msg = rep.status();
    if (status_msg.config_set()) {
      status[0] = 1;
    }
    if (status_msg.mesh_set()) {
      status[1] = 1;
    }
    if (status_msg.policy_set()) {
      status[2] = 1;
    }
    return status;
  }

  double* GetPolicy(int planner_num) {
    pb::RegisterPlannerReqMsg req;
    pb::RegisterPlannerRepMsg rep;
    req.set_req_node_name(node_name_);
    node_.call_rpc("Sim" + std::to_string(planner_num)
                   + "/GetPolicy", req, rep);
    // std::cout<<"Got a policy!!"<<std::endl;
    pb::PlannerPolicyMsg policy_msg = rep.policy();
    double* policy = TurnPolicyIntoArray(policy_msg);
    return policy;
  }

  double* GetMotionSample(int planner_num) {
    // This first 'policy' is a fail case
    // It returns -1 when we don't get a policy.
    double* policy = new double[1];
    policy[0] = -1;
    return policy;
  }


  double* GetSpline(int planner_num) {
    pb::RegisterPlannerReqMsg req;
    pb::RegisterPlannerRepMsg rep;
    req.set_req_node_name(node_name_);
    node_.call_rpc("Sim" + std::to_string(planner_num)
                   + "/GetSpline", req, rep);
    pb::PlannerSplineMsg spline_msg = rep.spline();
    // x and y points, and then a [x,y,th,v] goal pose.
    int spline_size = spline_msg.x_values().size();
    int array_size = (2 * spline_size) + 5;
    double* spline_params = new double[array_size];
    int counter = 1;
    spline_params[0] = spline_size;
    for (int ii = 0; ii < spline_size; ii++) {
      spline_params[counter] = spline_msg.x_values(ii);
      counter++;
    }
    for (int ii = 0; ii < spline_size; ii++) {
      spline_params[counter] = spline_msg.y_values(ii);
      counter++;
    }
    for (int ii = 0; ii < 4; ii++) {
      spline_params[counter] = spline_msg.solved_goal_pose(ii);
      counter++;
    }
    return spline_params;
  }


  ////////////////////////////////////////////////
  /// VECTOR MANIPULATORS
  /// #justmatlabthings

  double* Vector2Double(std::vector<double> vect) {
    double* new_doub = new double[vect.size()];
    for (int ii = 0; ii < vect.size(); ii++) {
      new_doub[ii] = vect.at(ii);
    }
    return new_doub;
  }

  std::vector<double> Double2Vector(double* doub, int size) {
    std::vector<double> new_vect;
    for (int ii = 0; ii < size; ii++) {
      new_vect.push_back(doub[ii]);
    }
    return new_vect;
  }

  ///////  ///////

  std::vector< std::vector<double> > TurnPolicyIntoVector(
      pb::PlannerPolicyMsg buffer) {
    std::vector< std::vector<double> > policy;
    std::vector<double> force;
    std::vector<double> phi;
    std::vector<double> time;
    // Force
    for (int ii = 0; ii < buffer.force_size(); ii++) {
      force.push_back(buffer.force(ii));
    }
    // Phi
    for (int ii = 0; ii < buffer.phi_size(); ii++) {
      phi.push_back(buffer.phi(ii));
    }
    // Time
    for (int ii = 0; ii < buffer.time_size(); ii++) {
      time.push_back(buffer.time(ii));
    }
    policy.push_back(force);
    policy.push_back(phi);
    policy.push_back(time);
    return policy;
  }

  double* TurnPolicyIntoArray(pb::PlannerPolicyMsg buffer) {
    // Array Size = time vector + force vector + phi vector + length of
    //   these vectors, so MATLAB knows + start state + goal state
    int array_size = buffer.time_size() + buffer.force_size() +
        buffer.phi_size() + 1;
    double* policy = new double[array_size];
    // Space 0 gives us the length of each column vector.
    policy[0] = buffer.force_size();
    int counter = 1;
    // Force
    for (int ii = 0; ii < buffer.force_size(); ii++) {
      policy[counter] = buffer.force(ii);
      counter++;
    }
    // Phi
    for (int ii = 0; ii < buffer.phi_size(); ii++) {
      policy[counter] = buffer.phi(ii);
      counter++;
    }
    // Time
    for (int ii = 0; ii < buffer.time_size(); ii++) {
      policy[counter] = buffer.time(ii);
      counter++;
    }
    return policy;
  }

  //////////////
  /// MEMBER VARIABLES
  //////////////

  node::node node_;
  std::string node_name_;
  pb::PlannerPolicyMsg policy_;

};

#endif // _PLANNERMASTER_H
