#ifndef __NODE_WRAPPER_
#define __NODE_WRAPPER_

#define PI 3.14159

// This is BAD FORM, but whatever. Time is of the essence.
#include <mat.h>

#include <PbMsgs/BVP.pb.h>
#include <Node/Node.h>
#include <unistd.h>
#include <cstdlib>

using namespace std;

class CPPMaster{
public:

  /// CONSTRUCTOR
  /// Creates all of our sims, and then assigns them all names.
  CPPMaster(int num_sims, std::string dir_to_sim);

  ////////////////////////
  /// SETTING OUR GOAL, GETTING OUR CONTROL
  ////////////////////////

  // SendBVP now takes in a filename, and is going to create
  // everything out of that.
  bool CreateBVP(int tau, std::string file_name,
               std::vector<double> goal_pt);
  // We receive commands from each of our Sims in turn.
  std::vector<std::vector<double> > ReceiveCommands(int sim_num);

  ////////////////////////
  /// MISC FUNCTIONS
  ////////////////////////

  // Uses the int passed as an index, and passes back the next goal config.
  std::vector< std::vector<double> > TurnPolicyIntoVector(
      pb::BVP_policy buffer);
  std::vector<double> GetNextBVP(int cur_pol);
  bool SavePolicyToMat(std::vector< std::vector<double> > policy, int tau);

  node::node node_;
  pb::BVP_params params_;
  pb::BVP_policy policy_;
  // TODO:
  // For now, the start state is always the same
  // After proof of concept, change the start veocity
  // After that... change all configurations of start.
  std::vector< std::vector<double> > goal_states_;

};

#endif // __NODE_WRAPPER_
