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
#include <boost/bind.hpp>
#include <Eigen/Eigen>                         // for vector math

#include "Utils/CVarHelpers.h"                 // for parsing Eigen Vars as CVars
#include <CVars/CVar.h>                        // for GLConsole
#include <Utils/GetPot>                        // for command line parsing

// Node used for communication between RP and any devices
#include <Node/Node.h>

#include "Managers/NetworkManager.h"           // for managing the Network
#include "Managers/RobotsManager.h"            // for managing all robots
#include "Managers/SimDeviceManager.h"         // for managing all the SimDevices

#include <URDFParser/URDF_Parser.h>            // for parsing URDF file

#include <SimRobots/SimRobot.h>                // for managing the User's robot
#include "SimRobots/SimWorld.h"

// for communicating between the Physics Engine and ModelGraph
#include <ModelGraph/ModelGraphBuilder.h>

// A controller (SimBA use only; does not use driver format)
#include <SimDevices/Controller/PoseController.h>

class LocalSim{

public:

  ///////////////////////////////////////////////////////////////////
  //member variables
  std::string                 m_sLocalSimName;
  ModelGraphBuilder           m_Scene;
  SimRobot                    m_SimRobot;         // user's robot. will be delete in final version of robot LocalSim (as we are not going to key control main robot in LocalSim)
  SimWorld                    m_SimWorld;
  SimDeviceManager            m_SimDeviceManager;
  RobotsManager               m_RobotManager;
  NetworkManager              m_NetworkManager;
  URDF_Parser                 m_Parser;

  /// Constructor
  LocalSim( const string &sLocalSimName,
            const std::string& sRobotURDF,
            const std::string& sWorldURDF,
            const std::string& sServerName,
            const std::string& sPoseFileName);

  ///Functions
  void ApplyCameraPose(Eigen::Vector6d dPose);
  void ApplyPoseToEntity(string sName, Eigen::Vector6d dPose);
  void StepForward(bool debug);
  bool SetImagesToWindow(SceneGraph::ImageView& LSimCamWnd,
                         SceneGraph::ImageView& RSimCamWnd );

};

/***************
 *The code below was used to move the camera in-program. We should be able to
 *do this through HAL, so I'm taking it out. For now.
 *********/


//////////////////////
///// INPUT KEYS
//////////////////////

//void LocalSim::LeftKey(){
//  string sMainRobotName = m_SimRobot->GetRobotName();

//  // update RGB camera pose
//  string sNameRGBCam   = "RGBLCamera@" + sMainRobotName;

//  Eigen::Vector6d dPoseRGB = _T2Cart(m_SimDeviceManager.GetSimCam(sNameRGBCam)->GetCameraPose() );
//  dPoseRGB(5,0) = dPoseRGB(5,0) - 0.1;
//  m_SimDeviceManager.GetSimCam(sNameRGBCam)->UpdateByPose(_Cart2T(dPoseRGB));

//  // update Depth camera pose
//  string sNameDepthCam = "DepthLCamera@"+sMainRobotName;

//  Eigen::Vector6d dPoseDepth =
//      _T2Cart(m_SimDeviceManager.GetSimCam(sNameDepthCam)->GetCameraPose() );
//  dPoseDepth(5,0) = dPoseDepth(5,0) - 0.1;
//  m_SimDeviceManager.
//      GetSimCam(sNameDepthCam)->UpdateByPose(_Cart2T(dPoseDepth));

//  //            string sMainRobotName = m_SimRobot->GetRobotName();
//  //            string sName = "RCamera@" + sMainRobotName;
//  //            Eigen::Vector6d dPose;
//  //            m_PhysEngine.GetEntity6Pose(sName, dPose);
//  //            dPose(0,0) = dPose(0,0) + 1;
//  //            m_PhysEngine.SetEntity6Pose(sName, dPose);
//}

//void LocalSim::RightKey(){
//  string sMainRobotName = m_SimRobot->GetRobotName();

//  // update RGB camera pose
//  string sNameRGBCam   = "RGBLCamera@" + sMainRobotName;

//  Eigen::Vector6d dPoseRGB = _T2Cart(m_SimDeviceManager.GetSimCam(sNameRGBCam)->GetCameraPose() );
//  dPoseRGB(5,0) = dPoseRGB(5,0) + 0.1;
//  m_SimDeviceManager.GetSimCam(sNameRGBCam)->UpdateByPose(_Cart2T(dPoseRGB));

//  // update Depth camera pose
//  string sNameDepthCam = "DepthLCamera@"+sMainRobotName;

//  Eigen::Vector6d dPoseDepth =
//      _T2Cart(m_SimDeviceManager.GetSimCam(sNameDepthCam)->GetCameraPose() );
//  dPoseDepth(5,0) = dPoseDepth(5,0) + 0.1;
//  m_SimDeviceManager.
//      GetSimCam(sNameDepthCam)->UpdateByPose(_Cart2T(dPoseDepth));
//}

//void LocalSim::ForwardKey(){
//  //  you should update the pose of the rig and then the poses of the cameras would always be relative to the rig
//  string sMainRobotName = m_SimRobot->GetRobotName();

//  // update RGB camera pose
//  string sNameRGBCam   = "RGBLCamera@" + sMainRobotName;

//  Eigen::Vector6d dPoseRGB = _T2Cart( m_SimDeviceManager.GetSimCam(sNameRGBCam)->GetCameraPose() );
//  dPoseRGB(1,0) = dPoseRGB(1,0) + 1;
//  m_SimDeviceManager.GetSimCam(sNameRGBCam)->UpdateByPose(_Cart2T(dPoseRGB));

//  // update Depth camera pose
//  string sNameDepthCam = "DepthLCamera@"+sMainRobotName;

//  Eigen::Vector6d dPoseDepth = _T2Cart(m_SimDeviceManager.GetSimCam(sNameDepthCam)->GetCameraPose() );
//  dPoseDepth(1,0) = dPoseDepth(1,0) + 1;
//  m_SimDeviceManager.GetSimCam(sNameDepthCam)->UpdateByPose(_Cart2T(dPoseDepth));
//}

//void LocalSim::ReverseKey(){
//  string sMainRobotName = m_SimRobot->GetRobotName();

//  // update RGB camera pose
//  string sNameRGBCam   = "RGBLCamera@" + sMainRobotName;

//  Eigen::Vector6d dPoseRGB = _T2Cart(m_SimDeviceManager.GetSimCam(sNameRGBCam)->GetCameraPose() );
//  dPoseRGB(1,0) = dPoseRGB(1,0) - 1;
//  m_SimDeviceManager.GetSimCam(sNameRGBCam)->UpdateByPose(_Cart2T(dPoseRGB));

//  // update Depth camera pose
//  string sNameDepthCam = "DepthLCamera@"+sMainRobotName;

//  Eigen::Vector6d dPoseDepth = _T2Cart(m_SimDeviceManager.GetSimCam(sNameDepthCam)->GetCameraPose() );
//  dPoseDepth(1,0) = dPoseDepth(1,0) - 1;
//  m_SimDeviceManager.
//      GetSimCam(sNameDepthCam)->UpdateByPose(_Cart2T(dPoseDepth));
//}



#endif // LocalSim_H
