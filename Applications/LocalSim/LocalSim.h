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
#include "Utils/CVarHelpers.h"                 // for parsing Eigen as CVars
#include <CVars/CVar.h>                        // for GLConsole
#include <Utils/GetPot>                        // for command line parsing
#include <URDFParser/TinyXMLTool.h>
#include <miniglog/logging.h>

// Node is used for communication between RP and any devices
#include <Node/Node.h>

#include "Managers/NetworkManager.h"           // for managing the Network
#include "Managers/RobotsManager.h"            // for managing all robots
#include <URDFParser/URDFParser.h>            // for parsing URDF file
#include <SimRobots/SimRobot.h>                // for managing the User's robot
#include "SimRobots/SimWorld.h"
#include "SimDevices/SimDevices.h"             // for managing the SimDevices

// for communicating between the Physics Engine and ModelGraph
#include <ModelGraph/ModelGraphBuilder.h>

class LocalSim{

public:

  ///////////////////////////////////////////////////////////////////
  //member variables
  std::string                 local_sim_name_;
  ModelGraphBuilder           scene_;
  std::shared_ptr<SimRobot> sim_robot_;
  std::shared_ptr<SimWorld> sim_world_;
  std::shared_ptr<SimDevices> sim_devices_;
  RobotsManager               robot_manager_;
  NetworkManager              network_manager_;
  URDFParser*                 parser_;
  bool                        render_option_;

  /// Constructor
  LocalSim( const string& local_sim_name,
            const string& sRobotURDF,
            const string& sWorldURDF,
            const string& sServerName,
            int debug_level);

  ///Functions
  void StepForward();
  bool SetImagesToWindow(SceneGraph::ImageView& LSimCamWnd,
                         SceneGraph::ImageView& RSimCamWnd );

};


#endif // LocalSim_H
