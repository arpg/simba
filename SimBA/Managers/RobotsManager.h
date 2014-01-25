// Manages all robots, includes user's robot and all other players' robots
// (when used in StateKeeper). Right now, we only support one user controlling
// one robot in his proxy. This robot is considered the 'main' robot.
// We should always createa and access SimRobot via RobotManager.

#ifndef ROBOTSMANAGER_H
#define ROBOTSMANAGER_H

#include <iostream>
#include <vector>
#include <Eigen/Eigen>             // for vector math
#include <NodeMessages.pb.h>
#include <SimRobots/SimRobot.h>
// to communicate between
//the physics engine and the ModelGraph
#include <ModelGraph/ModelGraphBuilder.h>
#include <tinyxml2.h>

using namespace std;

class RobotsManager
{

public:

  map<string, SimRobot*>            m_mSimRobotsList;  // the first robot join the list will be user's robot.
  WorldFullStateMsg                 m_WorldFullState;  // world state (pose, velocity) message. inlude the state of bodies.
  ModelGraphBuilder                 m_Scene;
  string                            m_sProxyName;
  string                            m_sMainRobotName;
  bool                              m_bStateKeeperOn;

  void Init(string sProxyName, string sServerName, ModelGraphBuilder &Scene);
  bool BuildRobot(XMLDocument& doc, string sProxyName);
  void DeleteRobot(string sRobotName);
  void UpdateWorldFullState(WorldFullStateMsg worldfullstate);
  void ApplyWorldFullStateOnAllPlayers();
  void ApplyWorldFullState();
  void DrawAllRobotsPoseAxis();
  void GenPoseAxis(Eigen::Vector6d &Pose, Eigen::Vector6d &AxisX, Eigen::Vector6d &AxisY, Eigen::Vector6d &AxisZ);
  SimRobot* GetMainRobot();
  SimRobot* GetRobot(string sRobotName);

};

#endif
