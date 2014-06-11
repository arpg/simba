#ifndef _PLANNERMASTER_H
#define _PLANNERMASTER_H

/*
 * File:   PlannerMaster.h
 * Author: bminortx
 * This wrapper is designed to facilitate the use of Node, in otder to pass
 * commands to PlannerLib
 */

#include <unistd.h>
#include <cstdlib>
#include "Node.h"
// Our protobuf sources - I couldn't find a successful way to import these
// using the makefile, so #includes had to do.
#include <BVP.pb.h>
#include "/Users/Trystan/Code/simba/build/SimBA/PB_Headers/BVP.pb.cc"



class PlannerMaster{
 public:

  /// CONSTRUCTOR
  PlannerMaster() {
    node_.init("MATLAB");
  }

  void StartConnections(int num_sims, std::string dir_to_sim){
    std::cout<<"MATLAB is successfully advertizing 'BVP'"<<std::endl;
    /// I think this should work...
    for(int i = 0; i < num_sims; i++) {
      std::string sim_name = "Sim"+std::to_string(i);
      std::string new_proc_name = dir_to_sim + " " + sim_name;
      node_.advertise("BVP"+std::to_string(i));
      while(!node_.subscribe(sim_name + "/CheckNeed")) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      while(!node_.subscribe(sim_name + "/CheckSolved")) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      while(!node_.subscribe(sim_name + "/Policy")) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      std::cout << std::endl << "MATLAB is subscribed to '" + sim_name
          + "/Policy'" << std::endl;
    }
  }

  ////////////////////////
  /// CHECK STATUS OF SIMS
  ////////////////////////

  double* CheckSimStatus(int sim_number){
    // give and take should never both be true.
    double* status = new double[2];
    status[0] = 0;
    status[1] = 0;
    pb::BVP_check sim_needs_bvp;
    pb::BVP_check sim_solved;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (node_.receive("Sim" + std::to_string(sim_number) + "/CheckNeed",
                     sim_needs_bvp)) {
      bool give = sim_needs_bvp.need();
      if (give == true) {
        status[0] = 1;
        status[1] = 0;
        return status;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (node_.receive("Sim" + std::to_string(sim_number) + "/CheckSolved",
                     sim_solved)) {
      bool take = sim_needs_bvp.need();
      if (take == true) {
        status[0] = 0;
        status[1] = 1;
        return status;
      }
    }
    return status;
  }

  ////////////////////////
  /// SENDING/RECEIVING COMMANDS
  ////////////////////////

  void SendBVP(int sim_number, int tau, double* x_data, double* y_data,
               double* z_data, int row_count, int col_count,
               double* start_point, double* goal_point) {
    // I don't know how else to do this, 'cause I'm ig'nant.
    params_.Clear();
    for (int ii = 0; ii < 4; ii++) {
      params_.add_start_param(start_point[ii]);
      params_.add_goal_param(goal_point[ii]);
    }
    for (int ii = 0; ii < (row_count * col_count); ii++) {
      params_.add_x_data(x_data[ii]);
      params_.add_y_data(y_data[ii]);
      params_.add_z_data(z_data[ii]);
    }
    params_.set_col_count(col_count);
    params_.set_row_count(row_count);
    params_.set_tau(tau);
    while (!node_.publish("BVP"+std::to_string(sim_number), params_)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  ////////////

  double* ReceivePolicy(int sim_num) {
    // This first 'policy' is a fail case
    // It returns -1 when we don't get a policy.
    double* policy = new double[1];
    policy[0] = -1;
    int count = 0;
    while (!node_.receive("Sim" + std::to_string(sim_num) + "/Policy", policy_)
           && count < 100) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      count++;
    }
    if (count < 100) {
      double* policy = TurnPolicyIntoArray(policy_);
      return policy;
    }
    return policy;
  }

  ////////////

  // Used in URDF_Parser.cpp in SimBA
  void SendHeightmap(double* x_data, double* y_data,
                     double* z_data, int row_count, int col_count) {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    pb::Heightmap map;
    for (int ii = 0; ii < (row_count * col_count); ii++) {
      map.add_x_data(x_data[ii]);
      map.add_y_data(y_data[ii]);
      map.add_z_data(z_data[ii]);
    }
    map.set_col_count(col_count);
    map.set_row_count(row_count);
    node_.advertise("Heightmap");
    std::this_thread::sleep_for(std::chrono::seconds(5));
    while (!node_.publish("Heightmap", map)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  ////////////////////////
  /// VECTOR MANIPULATORS
  /// #justmatlabthings
  ////////////////////////

  double* Vector2Double(std::vector<double> vect) {
    double* new_doub = new double[vect.size()];
    for (int ii = 0; ii < vect.size(); ii++) {
      new_doub[ii] = vect.at(ii);
    }
    return new_doub;
  }

  ///////

  std::vector<double> Double2Vector(double* doub, int size) {
    std::vector<double> new_vect;
    for(int ii = 0; ii < size; ii++) {
      new_vect.push_back(doub[ii]);
    }
    return new_vect;
  }

  ///////

  std::vector< std::vector<double> > TurnPolicyIntoVector(
      pb::BVP_policy buffer) {
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

  ///////

  double* TurnPolicyIntoArray(pb::BVP_policy buffer){
    // Array Size = time vector + force vector + phi vector + length of
    //   these vectors, so MATLAB knows + start state + goal state
    int array_size = buffer.time_size() + buffer.force_size() +
        buffer.phi_size() + 1 + 4 + 4;
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
    for (int ii = 0; ii < 4; ii++) {
      policy[counter] = buffer.start_param(ii);
      counter++;
    }
    for (int ii = 0; ii < 4; ii++) {
      policy[counter] = buffer.goal_param(ii);
      counter++;
    }
    return policy;
  }

  //////////////
  /// MEMBER VARIABLES
  //////////////

  node::node node_;
  pb::BVP_params params_;
  pb::BVP_policy policy_;

};

#endif // _PLANNERMASTER_H
