#ifndef NETWORKMANAGER_H
#define NETWORKMANAGER_H

#include <stdlib.h>
#include <iostream>
#include <Node/Node.h>
#include <SimDevices/SimDevices.h>
#include <URDFParser/URDF_Parser.h>
#include <Managers/RobotsManager.h>

/// Node messages for our controllers
#include <NodeMessages.pb.h>
#include <Camera.pb.h>
#include <NodeCamMessage.pb.h>
#include <NodeCar.pb.h>
#include <Network/WorldState.h>

using namespace std;

/*****************************************************
  * NETWORK MANAGER
  * NetworkManager manages two connections from LocalSim:
  *   1. Communicating between the Node system in HAL for controller/sensor
  *      input and output
  *   2. Communicating with a StateKeeper, if one is initialized.
  * If either one of these systems is disconnected, we just skip 'em.
  ****************************************************/

class NetworkManager
{
public:

  int m_iNodeClients; // num of Node clients that subscribe to LocalSim

  /// INITIALIZE NETWORK
  bool Init(string sLocalSimName, string sServerName, int verbocity=0);

  /// URI PARSERS
  std::map<string, string> ParseURI(string sURI);
  string CheckURI(string sURI);

  /// NODE FUNCTIONS
  void RegisterDevices(SimDevices* pSimDevices);
  void RegisterCamDevice(RegisterNodeCamReqMsg& mRequest,
                         RegisterNodeCamRepMsg & mReply);
  void RegisterControllerDevice(pb::RegisterControllerReqMsg& mRequest,
                                pb::RegisterControllerRepMsg & mReply);
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
  // STATIC FUNCTIONS CALLED BY HAL AND STATEKEEPER
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

  /// Register hal camera device in LocalSim. This RPC function is called by hal.
  /// Once we register a cam device, we can use the recv and publish method.

  static void _RegisterCamDevice(RegisterNodeCamReqMsg& mRequest,
                                 RegisterNodeCamRepMsg& mReply,
                                 void* pUserData){
    ((NetworkManager*)pUserData)->RegisterCamDevice(mRequest, mReply);
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

private:

  hal::node       m_Node;
  std::string     m_sLocalSimName;
  string          m_sServerName;
  int             m_verbocity;
  int             m_iTimeStep;
  boost::mutex    m_Mutex;
  SimDevices*     m_pSimDevices;
  RobotsManager*  m_pRobotsManager;

};

#endif // NETWORKMANAGER_H
