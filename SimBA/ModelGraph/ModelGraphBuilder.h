#ifndef MODELGRAPHBUILDER_H
#define MODELGRAPHBUILDER_H

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
            std::string sSimName, bool debug);

  /////////////////////////////////////////
  /// PHYSICS_ENGINE CONSTRUCTORS
  /////////////////////////////////////////

  void AssociateWorldPhysics(SimWorld m_SimWorld);

  void AssociatePhysicsShapes(SimRobot& m_SimRobot);

  void AssociatePhysicsConstraints(SimRobot& m_SimRobot);

  void AssociateRobotPhysics(SimRobot& m_SimRobot);

  /////////////////////////////////////////
  /// RENDER_ENGINE CONSTRUCTORS
  /////////////////////////////////////////

  void RenderWorldGraph(SimWorld m_SimWorld);

  void RenderRobotGraph(SimRobot m_SimRobot);

  /////////////////////////////////////////
  /// UPDATE THE SCENE
  /////////////////////////////////////////

  void UpdateScene();

  /// MEMBER VARIABLES
  Eigen::Vector6d m_PoseRW;
  PhysicsEngine m_Phys;
  RenderEngine m_Render;
  bool m_debug;

};

#endif // MODELGRAPHBUILDER_H
