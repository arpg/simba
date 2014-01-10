#ifndef NETWORKMANAGER_H
#define NETWORKMANAGER_H

#include <stdlib.h>
#include <iostream>
#include <Node/Node.h>
#include <Managers/SimDeviceManager.h>
#include <Managers/RobotsManager.h>
#include <NodeMessages.pb.h>
#include <Network/WorldState.h>

using namespace std;

class NetworkManager
{
public:
    int m_SubscribeNum; // num of clients that subscribe to RobotProxy

    /// INITIALIZE NODE NETWORK AND ALL DEVICES
    //-----------------------------------------------------
    bool initNetwork(string sProxyName, SimDeviceManager* pSimDeviceManager,
                     RobotsManager* pRobotsManager,  string sServerName,
                     int verbocity=0);
    bool initDevices();

    /// REGISTER AND DELETE ROBOTS FROM THE NETWORK
    //-----------------------------------------------------
    /// Used in StateKeeper and RobotProxy
    bool RegisterRobotProxyWithStateKeeper();
    static void _AddRobotByURDF(RobotProxyAddNewRobotReqMsg& mRequest,
                                RobotProxyAddNewRobotRepMsg& mReply,
                                void* pUserData);
    void AddRobotByURDF(RobotProxyAddNewRobotReqMsg& mRequest,
                        RobotProxyAddNewRobotRepMsg& mReply);
    static void _DeleteRobot(RobotProxyDeleteRobotReqMsg& mRequest,
                             RobotProxyDeleteRobotRepMsg& mReply,
                             void* pUserData);
    void DeleteRobot(RobotProxyDeleteRobotReqMsg& mRequest,
                     RobotProxyDeleteRobotRepMsg& mReply);
    bool PublishRobotFullStateToStateKeeper();
    bool ReceiveWorldFullStateFromStateKeeper();

    /// REGISTER AND DELETE DEVICES FROM THE SIMULATION
    //-----------------------------------------------------
    /// Used in HAL and RobotProxy
    static void _RegisterCamDevice(RegisterNode2CamReqMsg& mRequest,
                                   RegisterNode2CamRepMsg& mReply,
                                   void* pUserData);
    void RegisterCamDevice(RegisterNode2CamReqMsg& mRequest,
                           RegisterNode2CamRepMsg & mReply);
    static void _RegisterControllerDevice(RegisterControllerReqMsg& mRequest,
                                          RegisterControllerRepMsg& mReply,
                                          void* pUserData);
    void RegisterControllerDevice(RegisterControllerReqMsg& mRequest,
                                  RegisterControllerRepMsg & mReply);

    /// UPDATE AND PUBLISH INFO
    //-----------------------------------------------------
    bool UpdateNetWork();
    bool ReceiveControlInfo(string sDeviceName);
    bool ReceiveSimpleControllerInfo();
    bool ReceiveCarControllerInfo();
    bool PublishSimCamBySensor(string sCamName);
    bool PublishGPS(string sDeviceName);


private:
    hal::node                                    m_Node;
    std::string                                  m_sProxyName;
    string                                       m_sServerName;
    int                                          m_verbocity;
    int                                          m_iTimeStep;
    boost::mutex                                 m_Mutex;
    SimDeviceManager*                            m_pSimDeviceManager;
    RobotsManager*                               m_pRobotsManager;

};

#endif // NETWORKMANAGER_H
