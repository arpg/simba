#ifndef _SIMBAMASTER_H
#define _SIMBAMASTER_H

/*
 * File:   SimbaMaster.h
 * Author: bminortx
 * SimbaMaster connects MATLAB and SimBA::LocalSim. This allows two things:
 * 1. Simulate trajectories from MATLAB
 * 2. Test the results from a LocalPlanner optimization given to MATLAB
 */

#include <unistd.h>
#include <cstdlib>
#include "Node.h"
// Our protobuf sources - I couldn't find a successful way to import these
// using the makefile, so #includes had to do.
#include "PbMsgs/CarPlanner.pb.h"
#include "/Users/Trystan/Code/rslam/build/CoreDev/HAL/PbMsgs/CarPlanner.pb.cc"

class SimbaMaster{
 public:

  /// CONSTRUCTOR
  SimbaMaster() {
    std::string node_name = "SimbaMaster";
    node_.init(node_name);
  }

  /*********************************************************************
   * NODE FUNCTIONS
   **********************************************************************/

  void StartConnections() {
    // TODO: Modify this so that it uses an RPC call
    // node_.advertise("BVP"+std::to_string(i));
    // while(!node_.subscribe(sim_name+"/CheckSolved")) {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //   std::cout<<"=>";
    // }
  }

  /*********************************************************************
   * SETTERS/GETTERS
   **********************************************************************/

  void SetConfiguration(double* start_point, double* goal_point) {
    // I don't know how else to do this, 'cause I'm ig'nant.
    params_.Clear();
    for (int ii = 0; ii < 4; ii++) {
      params_.add_start_param(start_point[ii]);
      params_.add_goal_param(goal_point[ii]);
    }
    // TODO: Modify so that we have a good way to send 6d vectors
    // for (int ii = 0;ii<(row_count*col_count); ii++) {
    //   params_.add_x_data(x_data[ii]);
    //   params_.add_y_data(y_data[ii]);
    //   params_.add_z_data(z_data[ii]);
    // }
    // params_.set_col_count(col_count);
    // params_.set_row_count(row_count);
    // params_.set_tau(tau);
    while (!node_.publish("SetConfiguration", params_)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  ////////////

  // Used in URDF_Parser.cpp in SimBA
  void SetHeightmap(double* x_data, double* y_data,
                    double* z_data, int row_count, int col_count) {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    pb::Heightmap map;
    for (int ii = 0; ii < (row_count*col_count); ii++) {
      map.add_x_data(x_data[ii]);
      map.add_y_data(y_data[ii]);
      map.add_z_data(z_data[ii]);
    }
    map.set_col_count(col_count);
    map.set_row_count(row_count);
    node_.advertise("SetHeightmap");
    std::this_thread::sleep_for(std::chrono::seconds(5));
    while (!node_.publish("SetHeightmap", map)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  /*********************************************************************
   * INTERFACE FUNCTIONS
   **********************************************************************/

  void RunPolicy(double* force, double* phi, double* duration) {

    /// rpc call to physics engine as a hal::car, and send all the commands
    /// that we have to to simulator. There is already a template for this
    /// in PathPlannerTest. Use that

    // TODO: Modify so that we have a good way to send 6d vectors
    // for (int ii = 0; ii <(row_count*col_count); ii++) {
    //   params_.add_x_data(x_data[ii]);
    //   params_.add_y_data(y_data[ii]);
    //   params_.add_z_data(z_data[ii]);
    // }
    // params_.set_col_count(col_count);
    // params_.set_row_count(row_count);
    // params_.set_tau(tau);
    // while (!node_.publish("SetConfiguration", params_)) {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
  }

  ////////////

  void PlotMotionSample(double* x, double* y, double* z,
                        double* r, double* p, double* q) {
    /// rpc call to scenegraph, and add a waypoint at each of
    /// these points


    // TODO: Modify so that we have a good way to send 6d vectors
    // for (int ii = 0; ii <(row_count*col_count); ii++) {
    //   params_.add_x_data(x_data[ii]);
    //   params_.add_y_data(y_data[ii]);
    //   params_.add_z_data(z_data[ii]);
    // }
    // params_.set_col_count(col_count);
    // params_.set_row_count(row_count);
    // params_.set_tau(tau);
    // while (!node_.publish("SetConfiguration", params_)) {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
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

  double* TurnPolicyIntoArray(pb::BVP_policy buffer) {
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
  pb::BVP_params params_;
  pb::BVP_policy policy_;

};

#endif // _SIMBAMASTER_H
