#ifndef LOCALSIM_H
#define LOCALSIM_H

////////////////////////////////////////////
///
/// LOCALSIM
/// This is the Simulator that we use to interface between the SimDevices and
/// the GUI (run by SceneGraph)
///
/////////////////////////////////////////////
#include <iostream>
#include <Eigen/Eigen>                         // for vector math
#include "Utils/CVarHelpers.h"                 // for parsing Eigen Vars as CVars
#include <CVars/CVar.h>                        // for GLConsole
#include <Utils/GetPot>                        // for command line parsing

// Node is used for communication between RP and any devices
#include <Node/Node.h>

#include "Managers/NetworkManager.h"           // for managing the Network
#include "Managers/RobotsManager.h"            // for managing all robots


#include <URDFParser/URDF_Parser.h>            // for parsing URDF file

#include <SimRobots/SimRobot.h>                // for managing the User's robot
#include "SimRobots/SimWorld.h"
#include "SimDevices/SimDevices.h"             // for managing all the SimDevices

// for communicating between the Physics Engine and ModelGraph
#include <ModelGraph/ModelGraphBuilder.h>

class LocalSim{

public:

  ///////////////////////////////////////////////////////////////////
  //member variables
  std::string                 m_sLocalSimName;
  ModelGraphBuilder           m_Scene;
  SimRobot                    m_SimRobot;
  SimWorld                    m_SimWorld;
  SimDevices                  m_SimDevices;
  RobotsManager               m_RobotManager;
  NetworkManager              m_NetworkManager;
  URDF_Parser                 m_Parser;
  bool                        m_bRender;

  /// Constructor
  LocalSim( const string &sLocalSimName,
            const string& sRobotURDF,
            const string& sWorldURDF,
            const string& sServerName);

  ///Functions
  void StepForward();
  bool SetImagesToWindow(SceneGraph::ImageView& LSimCamWnd,
                         SceneGraph::ImageView& RSimCamWnd );

};


#endif // LocalSim_H
