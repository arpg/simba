#ifndef _MATLABMASTER_H
#define _MATLABMASTER_H

#define PI 3.14159

#include <unistd.h>
#include <cstdlib>

#include <mat.h>

#include <PB_Headers/BVP.pb.h>
#include <Node/Node.h>
#include <miniglog/logging.h>

class MatlabMaster{
 public:

  /// CONSTRUCTOR
  MatlabMaster(int num_sims);

  /// SETTING OUR GOAL, GETTING OUR CONTROL
  bool CreateBVP(int tau, std::string file_name,
                 std::vector<double> goal_pt);
  // We receive commands from each of our Sims in turn.
  std::vector<std::vector<double> > ReceiveCommands(int sim_num);

  /// MISC FUNCTIONS
  // Uses the int passed as an index, and passes back the next goal config.
  std::vector< std::vector<double> > TurnPolicyIntoVector(
      pb::BVP_policy buffer);
  std::vector<double> GetNextBVP(int cur_pol);
  bool SavePolicyToMat(std::vector< std::vector<double> > policy, int tau);

  node::node node_;
  pb::BVP_params params_;
  pb::BVP_policy policy_;
  std::vector< std::vector<double> > goal_states_;

};

#endif // _MATLABMASTER_H
