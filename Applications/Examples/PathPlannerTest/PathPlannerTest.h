#ifndef PATHPLANNERTEST_H
#define PATHPLANNERTEST_H

#include <iostream>



// Planning functions, from CarPlanner lib
#include "CarPlannerCommon.h"
#include "ApplyVelocitiesFunctor.h"
#include "LocalPlanner.h"
#include "miniglog/logging.h"
// GUI to debug the LocalPlanner
#include "PlannerGui.h"
#include "GLBulletDebugDrawer.h"

#include "Node/Node.h"
#include "URDFParser/URDF_Parser.h"
#include "PB_Headers/BVP.pb.h"
#include <HAL/Car/CarDevice.h>

// for communicating between the Physics Engine and ModelGraph
#include <ModelGraph/ModelGraphBuilder.h>

/// Keyboard commands
class KeyCarController{
 public:

  KeyCarController(std::string parameters, pb::BVP_policy policy){
    car_ = new hal::Car(parameters);
    // Keep the BVP_policy with us, just in case
    policy_ = policy;
    TurnPolicyIntoVector(policy);
    count_ = 0;
  }

  bool ApplyCommands(){
    double force = 0;
    double phi = 0;
    double command_time = 1.0/30.0;
    bool cont = true;
    if (count_ < policy_.time_size()) {
      force = force_.at(count_);
      phi = phi_.at(count_);
      command_time = time_.at(count_);
    } else {
      cont = false;
    }
    car_->ApplyCommand(force, phi, command_time);
    LOG(INFO) << force;
    LOG(INFO) << phi;
    LOG(INFO) << command_time;
    count_++;
    return cont;
  }

  void TurnPolicyIntoVector(pb::BVP_policy policy){
    // Force
    for (int ii=0; ii<policy.force_size(); ii++) {
      force_.push_back(policy.force(ii));
    }
    // Phi
    for (int ii=0; ii<policy.phi_size(); ii++) {
      phi_.push_back(policy.phi(ii));
    }
    // Time
    for (int ii=0; ii<policy.time_size(); ii++) {
      time_.push_back(policy.time(ii));
    }
  }

  /// MEMBER VARIABLES
  hal::Car* car_;
  pb::BVP_policy policy_;
  std::vector<double> force_;
  std::vector<double> phi_;
  std::vector<double> time_;
  double count_;

};

//////////////////////////////////////////////////

class PathPlannerTest
{
 public:

  //member variables
  std::string           sim_planner_name_;
  BulletCarModel*       car_model_;
  PlannerGui            planner_gui_;
  GLBulletDebugDrawer   m_GLDebugDrawer;
  LocalPlanner          m_snapper;
  CarParameterMap       m_VehicleParams;
  VehicleState          start_state_;
  VehicleState          goal_state_;
  MotionSample          m_msFinalPath;
  node::node            m_Node;
  pb::BVP_params        m_params;
  pb::BVP_policy        m_policy;
  int                   m_nTau; // Our bitstring that describes the map.
  bool                  need_BVP_;
  bool                  solved_BVP_;
  bool reset_;
  std::vector<double> start_;
  std::vector<double> goal_;
  std::string params_file_name_;
  HeightmapShape* heightmap_data_;
  // This is what we compare to our desired goal_param, from our policy
  std::vector<double> last_pose_;

  /// CONSTRUCTOR
  PathPlannerTest();
  ~PathPlannerTest();

  /// FUNCTIONS
  void Init(HeightmapShape* heightmap_data);
  void CheckNeed();
  void CheckSolved();
  pb::BVP_policy StartPolicy();
  bool InitMesh();
  void GroundStates();
  double* RaycastToGround(double id, double x, double y);
  void InitGoals();
  pb::BVP_policy SampleTrajectory();
  std::string GetNumber(std::string name);

};

#endif // PATHPLANNERTEST_H
