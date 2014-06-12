#ifndef NETWORKMANAGER_H
#define NETWORKMANAGER_H

#include <stdlib.h>
#include <iostream>

#include <Node/Node.h>
#include <miniglog/logging.h>

#include <SimDevices/SimDevices.h>
#include <URDFParser/URDF_Parser.h>
#include <Managers/RobotsManager.h>
#include <Network/WorldState.h>
#include <ModelGraph/Shape.h>

/// Node messages for our controllers
#include <NodeMessages.pb.h>
#include <Camera.pb.h>
#include <NodeCamMessage.pb.h>
#include <NodeCar.pb.h>
// To add shapes to our scenegraph in real-time
#include "PB_Headers/RenderShapes.pb.h"


using namespace std;

/*****************************************************
  * NETWORK MANAGER
  * NetworkManager manages two connections from LocalSim:
  *   1. Communicating between the Node system in HAL for controller/sensor
  *      input and output
  *   2. Communicating with a StateKeeper, if one is initialized.
  * If either one of these systems is disconnected, we just skip 'em.
  ****************************************************/

class NetworkManager {
 public:

  int m_iNodeClients; // num of Node clients that subscribe to LocalSim

  /// INITIALIZE NETWORK
  bool Init(string sLocalSimName, string sServerName, int debug_level);

  /// URI PARSERS
  std::map<string, string> ParseURI(string sURI);
  string CheckURI(string sURI);

  /// NODE FUNCTIONS
  void RegisterDevices(SimDevices* pSimDevices);
  void RegisterSensorDevice(RegisterNodeCamReqMsg& mRequest,
                            RegisterNodeCamRepMsg & mReply);
  void RegisterControllerDevice(pb::RegisterControllerReqMsg& mRequest,
                                pb::RegisterControllerRepMsg & mReply);
  void AddRenderObject(pb::RegisterRenderReqMsg& mRequest,
                       pb::RegisterRenderRepMsg & mReply);
  bool UpdateNetwork();
  bool ReceiveControllerInfo(string sDeviceName);
  bool PublishSimCamBySensor(string sCamBodyName);
  bool PublishGPS(string sDeviceName);


  /// STATEKEEPER FUNCTIONS
  bool RegisterRobot(RobotsManager* pRobotsManager);
  bool RegisterWithStateKeeper();
  void AddRobotByURDF(LocalSimAddNewRobotReqMsg& mRequest,
                      LocalSimAddNewRobotRepMsg& mReply);
  void DeleteRobot(LocalSimDeleteRobotReqMsg& mRequest,
                   LocalSimDeleteRobotRepMsg& mReply);
  bool PublishRobotToStateKeeper();
  bool ReceiveWorldFromStateKeeper();

  //////////////////////////////////////
  // RPC FUNCTIONS CALLED BY HAL AND STATEKEEPER
  //////////////////////////////////////

  // add a new robot by URDF (Called by StateKeeper)
  static void _AddRobotByURDF(LocalSimAddNewRobotReqMsg& mRequest,
                              LocalSimAddNewRobotRepMsg& mReply,
                              void* pUserData){
    ((NetworkManager*)pUserData)->AddRobotByURDF(mRequest, mReply);
  }

  //////////////////////////////

  static void _DeleteRobot(LocalSimDeleteRobotReqMsg& mRequest,
                           LocalSimDeleteRobotRepMsg& mReply,
                           void* pUserData){
    ((NetworkManager*)pUserData)->DeleteRobot(mRequest, mReply);
  }

  //////////////////////////////
  // Register hal camera device in LocalSim. This RPC function is called by hal.
  // Once we register a cam device, we can use the recv and publish method.

  static void _RegisterSensorDevice(RegisterNodeCamReqMsg& mRequest,
                                    RegisterNodeCamRepMsg& mReply,
                                    void* pUserData){
    ((NetworkManager*)pUserData)->RegisterSensorDevice(mRequest, mReply);
  }

  //////////////////////////////
  // Register controller device in LocalSim. This RPC function is called by hal.
  // Once we register a controller, LocalSim will need to subscribe to it and
  // then we can use recv and publish method to sync command between
  // controller and LocalSim.

  static void _RegisterControllerDevice(pb::RegisterControllerReqMsg& mRequest,
                                        pb::RegisterControllerRepMsg& mReply,
                                        void* pUserData){
    ((NetworkManager*)pUserData)->RegisterControllerDevice(mRequest, mReply);
  }

  //////////////////////////////
  // Register controller device in LocalSim. This RPC function is called by hal.

  static void _AddRenderObject(pb::RegisterRenderReqMsg& mRequest,
                               pb::RegisterRenderRepMsg& mReply,
                               void* pUserData){
    ((NetworkManager*)pUserData)->AddRenderObject(mRequest, mReply);
  }

 private:

  node::node      node_;
  int             debug_level_;
  std::string     local_sim_name_;
  string          server_name_;
  int             timestep_;
  SimDevices*     sim_devices_;
  RobotsManager*  robot_manager_;
  std::mutex      statekeeper_mutex_;

};

#endif // NETWORKMANAGER_H
