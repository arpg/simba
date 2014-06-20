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

  bool Init(const string& sim_name, const ModelGraphBuilder& scene,
            SimRobot& sim_robot, const string &statekeeper_option);
  bool ImportSimRobot( SimRobot& sim_robot );
  void DeleteRobot(string robot_name);
  void UpdateWorldFullState(WorldFullStateMsg worldfullstate);
  void ApplyWorldFullStateOnAllPlayers();
  void ApplyWorldFullState();
  void DrawAllRobotsPoseAxis();
  void GenPoseAxis(Eigen::Vector6d &Pose, Eigen::Vector6d &AxisX,
                   Eigen::Vector6d &AxisY, Eigen::Vector6d &AxisZ);
  SimRobot* GetMainRobot();
  SimRobot* GetRobot(string robot_name);

  // MEMBER VARIABLES

  // the first robot join the list will be user's robot.
  map<string, SimRobot*> sim_robots_map_;
  // world state (pose, velocity) message. inlude the state of bodies.
  WorldFullStateMsg world_state_;
  ModelGraphBuilder scene_;
  string sim_name_;
  // Name of Main Robot. This is actually the robot we can control
  string main_robot_name_;
  bool statekeeper_option_;

};

#endif
