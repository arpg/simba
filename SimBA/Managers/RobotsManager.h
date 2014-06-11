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
// the physics engine and the ModelGraph
#include <ModelGraph/ModelGraphBuilder.h>
#include <tinyxml2.h>

using namespace std;

class RobotsManager
{

public:

  bool Init(string& sProxyName, ModelGraphBuilder& Scene,
            SimRobot& mSimRobot, const string &sServerOption);
  bool ImportSimRobot( SimRobot& mSimRobot );
  void DeleteRobot(string sRobotName);
  void UpdateWorldFullState(WorldFullStateMsg worldfullstate);
  void ApplyWorldFullStateOnAllPlayers();
  void ApplyWorldFullState();
  void DrawAllRobotsPoseAxis();
  void GenPoseAxis(Eigen::Vector6d &Pose, Eigen::Vector6d &AxisX,
                   Eigen::Vector6d &AxisY, Eigen::Vector6d &AxisZ);
  SimRobot* GetMainRobot();
  SimRobot* GetRobot(string sRobotName);

  // MEMBER VARIABLES

  // the first robot join the list will be user's robot.
  map<string, SimRobot*> m_mSimRobotsList;
  // world state (pose, velocity) message. inlude the state of bodies.
  WorldFullStateMsg m_WorldFullState;
  ModelGraphBuilder m_Scene;
  string m_sProxyName;
  // Name of Main Robot. This is actually the robot we can control
  string m_sMainRobotName;
  bool m_bStateKeeperOn;

};

#endif
