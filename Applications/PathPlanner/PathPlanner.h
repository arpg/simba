#ifndef PATHPLANNER_H
#define PATHPLANNER_H

// Planning functions, from CarPlanner lib
#include "PlannerLib/CarPlannerCommon.h"
#include "PlannerLib/ApplyVelocitiesFunctor.h"
#include "PlannerLib/LocalPlanner.h"

#include "miniglog/logging.h"
#include "Node/Node.h"
#include "PB_Headers/BVP.pb.h"
// for communicating between the Physics Engine and ModelGraph
#include "ModelGraph/ModelGraphBuilder.h"

/******************************************
 /// PATH PLANNER
 /// This function acts as a data wrapper for the LocalPlanner.
 /// It takes three data structures:
 ///   1. The start configuration of the vehicle [x, y, yaw, vel].
 ///   2. The goal configuration [x, y, yaw, vel].
 ///   3. The data describing the mesh [row_ct, col_ct, X_array, Y_array,
 ///      Z_array]
 /// All of this data comes from the MATLAB wrapper program, and is
 /// passed through Node. The network connection here is key; it allows
 /// us to start several intsances of PathPlanner, and get more done, faster.
/******************************************/

class PathPlanner {
 public:

  /// CONSTRUCTOR
  PathPlanner();
  ~PathPlanner();

  /// FUNCTIONS
  void InitNode();
  void SetConfiguration(pb::RegisterPlannerReqMsg& mRequest,
                        pb::RegisterPlannerRepMsg& mReply);
  void SetHeightmap(pb::RegisterPlannerReqMsg& mRequest,
                    pb::RegisterPlannerRepMsg& mReply);
  void GetStatus(pb::RegisterPlannerReqMsg& mRequest,
                 pb::RegisterPlannerRepMsg& mReply);
  void GetPolicy(pb::RegisterPlannerReqMsg& mRequest,
                 pb::RegisterPlannerRepMsg& mReply);
  void GetMotionSample(pb::RegisterPlannerReqMsg& mRequest,
                       pb::RegisterPlannerRepMsg& mReply);
  void GetSpline(pb::RegisterPlannerReqMsg& mRequest,
                 pb::RegisterPlannerRepMsg& mReply);

  void GroundStates();
  void SolveBVP();
  std::string GetNumber(std::string name);
  void ResetBooleans();

  // NODE RPC FUNCTIONS
  // These correspond to functions in PlannerMaster.h
  static void _SetConfiguration(pb::RegisterPlannerReqMsg& mRequest,
                                pb::RegisterPlannerRepMsg& mReply,
                                void* pUserData){
    ((PathPlanner*)pUserData)->SetConfiguration(mRequest, mReply);
  }

  static void _SetHeightmap(pb::RegisterPlannerReqMsg& mRequest,
                            pb::RegisterPlannerRepMsg& mReply,
                            void* pUserData){
    ((PathPlanner*)pUserData)->SetHeightmap(mRequest, mReply);
  }

  static void _GetStatus(pb::RegisterPlannerReqMsg& mRequest,
                         pb::RegisterPlannerRepMsg& mReply,
                         void* pUserData){
    ((PathPlanner*)pUserData)->GetStatus(mRequest, mReply);
  }

  static void _GetPolicy(pb::RegisterPlannerReqMsg& mRequest,
                         pb::RegisterPlannerRepMsg& mReply,
                         void* pUserData){
    ((PathPlanner*)pUserData)->GetPolicy(mRequest, mReply);
  }

  static void _GetMotionSample(pb::RegisterPlannerReqMsg& mRequest,
                               pb::RegisterPlannerRepMsg& mReply,
                               void* pUserData){
    ((PathPlanner*)pUserData)->GetMotionSample(mRequest, mReply);
  }

  static void _GetSpline(pb::RegisterPlannerReqMsg& mRequest,
                         pb::RegisterPlannerRepMsg& mReply,
                         void* pUserData){
    ((PathPlanner*)pUserData)->GetSpline(mRequest, mReply);
  }

  /// MEMBER VARIABLES
  // PlannerLib things
  std::unique_ptr<BulletCarModel> car_model_;
  CarParameterMap m_VehicleParams;
  VehicleState start_state_;
  VehicleState goal_state_;
  MotionSample trajectory_;
  // Spline parameters
  Eigen::VectorXd spline_x_values_;
  Eigen::VectorXd spline_y_values_;
  Eigen::Vector4d spline_goal_pose_;

  std::string params_file_name_;
  //Node things
  node::node node_;
  std::string planner_name_;
  std::string master_node_name_;
  int debug_level_;
  std::vector<double> start_;
  std::vector<double> goal_;

  // Our bitstring that describes the map.
  int  map_tau_;
  // booleans
  bool config_set_;
  bool mesh_set_;
  bool policy_set_;
  bool policy_delivered_;

};

#endif // PATHPLANNER_H
