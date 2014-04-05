#ifndef __NODE_WRAPPER_
#define __NODE_WRAPPER_

/*
 * File:   node_wrapper.h
 * Author: bminortx
 * This wrapper is designed to facilitate the use of Node, in otder to pass
 * commands to SimBA.
 */

#include <PbMsgs/BVP.pb.h>
#include <Node/Node.h>
#include <unistd.h>
#include <cstdlib>

class NodeWrapper{
public:

  /// CONSTRUCTOR
  NodeWrapper(int num_sims, std::string dir_to_sim){
    std::cout<<"WOO"<<std::endl;
    node_.init("MATLAB");
    node_.advertise("BVP");
    /// I think this should work...
    for(int i = 0; i<num_sims; i++){
      std::cout<<"WOO"<<std::endl;
      int pid = fork();
      if(pid==0){
        std::cout<<"WOO"<<std::endl;
        std::string sim_name = "Sim"+std::to_string(i);
        sim_name = dir_to_sim+" "+sim_name;
        std::system(sim_name.c_str());
      }
    }
  }

  ////////////////////////
  /// SETTING OUR GOAL
  ////////////////////////

  bool SendBVP(int sim_num, double* start_point, double* goal_point,
              int row_count, int col_count, int tau,
              double* x_data, double* y_data, double* z_data){
    pb::RegisterBVPReqMsg req;
    pb::RegisterBVPRepMsg rep;
    node_.call_rpc("Sim"+std::to_string(sim_num)+"/StartPolicy", req, rep);
    pb::BVP_check need_BVP;
    while (!node_.receive("Sim"+std::to_string(sim_num)+" /NeedBVP",
                          need_BVP)) {
    }
    if (need_BVP.need()==true) {
      // Not sure if this works...
      // TODO: Push all of the doubles on one at a time.
      params_.set_start_param(0, *start_point);
      params_.set_goal_param(0, *goal_point);
      params_.set_col_count(col_count);
      params_.set_row_count(row_count);
      params_.set_x_data(0, *x_data);
      params_.set_y_data(0, *y_data);
      params_.set_z_data(0, *z_data);
      params_.set_tau(tau);
      //Play around with this... it may not be enough.
      while (!node_.publish("BVP", params_)) {

      }
      return true;
    }
    return false;
  }

  ////////////////////////
  /// SENDING/RECEIVING COMMANDS
  ////////////////////////
  double* ReceiveCommands(int sim_num){
    pb::RegisterBVPReqMsg req;
    pb::RegisterBVPRepMsg rep;
    node_.call_rpc("Sim"+std::to_string(sim_num)+"/RetrievePolicy", req, rep);
    pb::BVP_check solved_BVP;
    while (!node_.receive("Sim"+std::to_string(sim_num)+" /SolvedBVP",
                          solved_BVP)) {
    }
    if (solved_BVP.need()==true) {
      while (!node_.receive("Sim"+std::to_string(sim_num)+"/Policy", policy_)) {
      // Keep trying!
      }

    }

  }

  hal::node node_;
  pb::BVP_params params_;
  pb::BVP_policy policy_;

};

#endif // __NODE_WRAPPER_
