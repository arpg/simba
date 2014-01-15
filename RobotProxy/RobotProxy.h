#ifndef ROBOTPROXY_H
#define ROBOTPROXY_H

////////////////////////////////////////////
///
/// ROBOT PROXY
/// This is the Simulator that we use to interface between the SimDevices and
/// the GUI (run by SceneGraph)
///
/////////////////////////////////////////////

#include <iostream>
#include <boost/bind.hpp>
#include <Eigen/Eigen>                         // for vector math
#include <SceneGraph/SceneGraph.h>             // for OpenGL SceneGraph


#include <Managers/NetworkManager.h>         // for managing the Network
#include "Managers/WorldManager.h"             // for managing the World
#include "Managers/SimDeviceManager.h"         // for managing all the SimDevices
                                               // of the Robot we control

#include <URDFParser/RobotProxyURDFParser.h>   // for parsing URDF file


#include "Utils/CVarHelpers.h"                 // for parsing Eigen Vars as CVars
#include <CVars/CVar.h>                        // for GLConsole
#include <Utils/GetPot>                        // for command line parsing



#include "Managers/RobotsManager.h"            // for managing all robots
#include <SimRobots/SimRobot.h>                // for managing the User's robot
#include <ModelGraph/PhyModelGraphAgent.h>     // for communicating between the
                                               // Physics Engine and ModelGraph
#include <SimDevices/Controller/PoseController.h>
#include <Node/Node.h>                         // Node used for communication
                                               // between RP and any devices

class RobotProxy{

public:

  ///////////////////////////////////////////////////////////////////
  //member variables
  std::string                 m_sProxyName;
  std::string                 m_sRobotURDFFile;
  std::string                 m_sWorldURDFFile;

  SceneGraph::GLLight         m_light;
  SceneGraph::GLBox           m_ground;
  SceneGraph::GLGrid          m_grid;
  SceneGraph::GLSceneGraph&   m_rSceneGraph;
  SceneGraph::GLMesh          m_Map;                 // mesh for the world.

  Render                      m_Render;
  SimRobot*                   m_pMainRobot;            // user's robot. will be delete in final version of robot proxy (as we are not going to key control main robot in proxy)
  WorldManager                m_WorldManager;
  SimDeviceManager            m_SimDeviceManager;
  RobotsManager               m_RobotManager;
  NetworkManager              m_NetworkManager;
  PhyModelGraphAgent          m_PhyMGAgent;          // for one sim proxy, there is one PhyAgent
  PoseController              m_SimpPoseController;

  hal::node                   m_Node;

  /// Constructor
  RobotProxy(
      SceneGraph::GLSceneGraph& glGraph,
      const std::string& sProxyName,
      const std::string& sRobotURDF,
      const std::string& sWorldURDF,
      const std::string& sServerName,
      const std::string& sPoseFileName);

  ///Functions
  void InitReset();
  void ApplyCameraPose(Eigen::Vector6d dPose);
  void ApplyPoseToEntity(string sName, Eigen::Vector6d dPose);
  void StepForward();
  bool SetImagesToWindow(SceneGraph::ImageView& LSimCamWnd,
                         SceneGraph::ImageView& RSimCamWnd );

  ///Direction Keys, hardcoded to move our robot.
  void LeftKey();
  void RightKey();
  void ForwardKey();
  void ReverseKey();

};

#endif // ROBOTPROXY_H
