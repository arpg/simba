#ifndef SIMPLANNER_H
#define SIMPLANNER_H

////////////////////////////////////////////
/// SIMPLANNER
/// This function acts as a data wrapper for the LocalPlanner.
/// It takes three data structures:
///   1. The start configuration of the vehicle [x, y, yaw, vel].
///   2. The goal configuration [x, y, yaw, vel].
///   3. The data describing the mesh [row_ct, col_ct, X_array, Y_array,
///      Z_array]
/// All of this data comes from the MATLAB wrapper program, and is
/// passed through Node. The network connection here is key; it allows
/// us to start several intsances of SimPlanner, and get more done, faster.
/////////////////////////////////////////////

#include <iostream>

#include <PbMsgs/BVP.pb.h>
#include <Node/Node.h>

// Planning functions, from CarPlanner lib
#include "CarPlannerCommon.h"
#include "ApplyVelocitiesFunctor.h"
#include "LocalPlanner.h"

// for communicating between the Physics Engine and ModelGraph
#include <ModelGraph/ModelGraphBuilder.h>

class SimPlanner
{
public:

  ///////////////////////////////////////////////////////////////////
  //member variables
  std::string           sim_planner_name_;
  BulletCarModel*       m_CarModel;
  LocalPlanner          m_snapper;
  CarParameterMap       m_VehicleParams;
  VehicleState          m_vsStart;
  VehicleState          m_vsGoal;
  MotionSample          m_msFinalPath;
  node::node             m_Node;
  pb::BVP_params        m_params;
  pb::BVP_policy        m_policy;
  int                   m_nTau; // Our bitstring that describes the map.
  bool                  need_BVP_;
  bool                  solved_BVP_;
  bool reset_;
  std::vector<double> start_;
  std::vector<double> goal_;
  std::string params_file_name_;

  /// CONSTRUCTOR
  SimPlanner();
  ~SimPlanner();

  /// FUNCTIONS
  void Init();
  void CheckNeed();
  void CheckSolved();
  pb::BVP_policy StartPolicy(pb::BVP_params params);
  bool InitMesh(pb::BVP_params params);
  void GroundStates();
  double* RaycastToGround(double id, double x, double y);
  void InitGoals(pb::BVP_params params);
  pb::BVP_policy SampleTrajectory();
  std::string GetNumber(std::string name);

};

#endif // SIMPLANNER_H
