#ifndef PATHPLANNER_H
#define PATHPLANNER_H

////////////////////////////////////////////
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
/////////////////////////////////////////////

#include <iostream>

// Planning functions, from CarPlanner lib
#include "CarPlannerCommon.h"
#include "ApplyVelocitiesFunctor.h"
#include "LocalPlanner.h"

#include "miniglog/logging.h"
#include "Node/Node.h"
#include "PB_Headers/BVP.pb.h"
// for communicating between the Physics Engine and ModelGraph
#include <ModelGraph/ModelGraphBuilder.h>

class PathPlanner
{
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
void SolveBVP(pb::PlannerPolicyMsg& policy);
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
std::string planner_name_;
BulletCarModel* car_model_;
LocalPlanner m_snapper;
CarParameterMap m_VehicleParams;
VehicleState start_state_;
VehicleState goal_state_;
MotionSample final_motion_sample_;
std::string params_file_name_;
//Node things
node::node node_;
pb::PlannerPolicyMsg policy_;
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
