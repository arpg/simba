#ifndef NETWORKMANAGER_H
#define NETWORKMANAGER_H

#include <stdlib.h>
#include <iostream>
#include <Node/Node.h>
#include <Managers/SimDeviceManager.h>
#include <URDFParser/URDF_Parser.h>
#include <Managers/RobotsManager.h>
#include <NodeMessages.pb.h>
#include <Network/WorldState.h>

using namespace std;

/*****************************************************
  * NETWORK MANAGER
  * NetworkManager manages two connections from LocalSim:
  *  1. Communicating between the Node system in HAL for controller/sensor
  *     input and output
  *  2. Communicating with a StateKeeper, if one is initialized.
  * If either one of these systems is disconnected, we just skip 'em.
  ****************************************************/

class NetworkManager
{
public:

  int m_iNodeClients; // num of Node clients that subscribe to LocalSim

  ////////////////////
  // FUNCTIONS
  ////////////////////

  /// INITIALIZE NODE NETWORK AND ALL DEVICES
  //-----------------------------------------------------
  bool Init(string sProxyName, string sServerName, int verbocity=0);

  bool RegisterRobot(RobotsManager* pRobotsManager);
  void RegisterDevices(SimDeviceManager* pSimDeviceManager);

  /// REGISTER AND DELETE ROBOTS FROM THE NETWORK
  //-----------------------------------------------------
  /// Used in StateKeeper and LocalSim
  bool RegisterWithStateKeeper();
  void AddRobotByURDF(LocalSimAddNewRobotReqMsg& mRequest,
                      LocalSimAddNewRobotRepMsg& mReply);
  void DeleteRobot(LocalSimDeleteRobotReqMsg& mRequest,
                   LocalSimDeleteRobotRepMsg& mReply);
  bool PublishRobotToStateKeeper();
  bool ReceiveWorldFromStateKeeper();

  /// REGISTER AND DELETE DEVICES FROM THE SIMULATION
  //-----------------------------------------------------
  /// Used in HAL and LocalSim
  void RegisterCamDevice(RegisterNodeCamReqMsg& mRequest,
                         RegisterNodeCamRepMsg & mReply);

  void RegisterControllerDevice(RegisterControllerReqMsg& mRequest,
                                RegisterControllerRepMsg & mReply);

  string CheckURI(string sURI);

  /// UPDATE AND PUBLISH INFO
  //-----------------------------------------------------
  bool UpdateNetWork();
  bool ReceiveControlInfo(string sDeviceName);
  bool ReceiveSimpleControllerInfo();
  bool ReceiveCarControllerInfo();
  bool PublishSimCamBySensor(string sCamName);
  bool PublishGPS(string sDeviceName);


  //////////////////////////////////////
  // STATIC FUNCTIONS
  //////////////////////////////////////

  // add a new robot by URDF (Called by StateKeeper)
  static void _AddRobotByURDF(LocalSimAddNewRobotReqMsg& mRequest,
                              LocalSimAddNewRobotRepMsg& mReply,
                              void* pUserData){
    ((NetworkManager*)pUserData)->AddRobotByURDF(mRequest, mReply);
  }

  /////

  static void _DeleteRobot(LocalSimDeleteRobotReqMsg& mRequest,
                                           LocalSimDeleteRobotRepMsg& mReply,
                                           void* pUserData){
    ((NetworkManager*)pUserData)->DeleteRobot(mRequest, mReply);
  }

  /////

  /// Register hal camera device in Proxy. This RPC function is called by hal.
  /// Once we register a cam device, we can use the recv and publish method.

  static void _RegisterCamDevice(
      RegisterNodeCamReqMsg& mRequest,
      RegisterNodeCamRepMsg& mReply,
      void* pUserData){
    ((NetworkManager*)pUserData)->RegisterCamDevice(mRequest, mReply);
  }

  /////

  // Register controller device in LoaclSim. This RPC function is called by hal.
  // Once we register a controller, LocalSim will need to subscribe to it and
  // then we can use recv and publish method to sync command between
  // controller and proxy.

  static void _RegisterControllerDevice(
      RegisterControllerReqMsg& mRequest,
      RegisterControllerRepMsg& mReply,
      void* pUserData){
    ((NetworkManager*)pUserData)->RegisterControllerDevice(mRequest, mReply);
  }




private:
  hal::node                                    m_Node;
  std::string                                  m_sLocalSimName;
  string                                       m_sServerName;
  int                                          m_verbocity;
  int                                          m_iTimeStep;
  boost::mutex                                 m_Mutex;
  SimDeviceManager*                            m_pSimDeviceManager;
  RobotsManager*                               m_pRobotsManager;

};

#endif // NETWORKMANAGER_H
