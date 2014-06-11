#ifndef MODELGRAPHBUILDER_H
#define MODELGRAPHBUILDER_H

#include <mutex>

#include <ModelGraph/RenderEngine.h>
#include <ModelGraph/PhysicsEngine.h>
#include <SimRobots/SimRobot.h>
#include <SimRobots/SimWorld.h>
#include <SimDevices/SimDevices.h>

class ModelGraphBuilder{
 public:

  /////////////////////////////////////////
  /// INITIALIZER FOR THE WORLD
  /////////////////////////////////////////

  void Init(SimWorld& m_WorldModel, SimRobot& m_SimRobot,
            SimDevices& m_SimDevices,
            std::string sSimName, bool debug,
            bool render, bool bEnableCameraView);

  /// PHYSICS_ENGINE CONSTRUCTORS
  void AssociateWorldPhysics(SimWorld m_SimWorld);
  void AssociatePhysicsShapes(SimRobot& m_SimRobot);
  void AssociatePhysicsConstraints(SimRobot& m_SimRobot);
  void AssociateRobotPhysics(SimRobot& m_SimRobot);
  void AssociateDevices(SimDevices& m_SimDevices);

  /// RENDER_ENGINE CONSTRUCTORS
  void RenderWorldGraph(SimWorld m_SimWorld);
  void RenderRobotGraph(SimRobot m_SimRobot);

  /// UPDATE THE SCENE
  void CheckForNewShapes();
  void UpdateScene();

  /// MEMBER VARIABLES
  Eigen::Vector6d robot_pose_;
  PhysicsEngine physics_engine_;
  RenderEngine render_engine_;
  SimWorld* world_model_;
  SimRobot* sim_robot_;
  bool debug_status_;
  bool render_status_;

};

#endif // MODELGRAPHBUILDER_H
