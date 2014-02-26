#ifndef STATEKEEPER_H
#define STATEKEEPER_H

#include <stdio.h>
#include <Node/Node.h>                   // for networking
#include <NodeMessages.pb.h>
#include <PB_Headers/SimMessages.pb.h>
#include <Utils/GetPot>                  // for command line parsing
#include <Utils/ConvertName.h>
#include <URDFParser/URDF_Parser.h>


class StateKeeper
{

public:
  hal::node                               m_Node;

  ///Constructor
  StateKeeper(string sStateKeeperName, string sWorldURDFFile);

  ///Functions
  void InitRobotPose();
  void RegisterLocalSim(RegisterLocalSimReqMsg& mRequest,
                          RegisterLocalSimRepMsg& mReply);
  bool CheckIfNeedToSendProxysURDF();
  bool CheckIfNeedToDeleteRobotInAllProxys();
  bool ReceiveWorldFullState();
  bool PublishWorldFullState();
  void ClearAllPreviousMessageIfNecessary();

  ////////////////////////////////////////////////////////////////////////

  // Register a LocalSim with the StateKeeper using its name.

  static void _RegisterLocalSim(RegisterLocalSimReqMsg& mRequest,
                                               RegisterLocalSimRepMsg& mReply,
                                               void* pUserData){
    ((StateKeeper*)pUserData)->RegisterLocalSim(mRequest, mReply);
  }

  /////////





private:
  string                                  m_sWorldURDFFileName;
  bool                                    m_bSendURDFtoProxys;  // flag if there is a new join proxy with robot. if true we will send this robot to all other proxy
  string                                  m_sLastJoinRobotName; // name of last joint robot. e.g. robot1@proxy1
  map<string,string>                      m_mNameList;          // <proxy name, robot name>
  map<string,URDFMsg*>                    m_mURDF;              // <robot name, robot URDFMsg> Robot Name example:  robot1@proxy1
  WorldFullStateMsg                       m_WorldFullState;     // a table of world state. Only StateKeeper can send this table to other robot.
  int                                     m_iTimeStep;          // time step
  vector<Eigen::Vector6d>                 m_vInitialPose;       // vector of init robot pose. read from word.xml file
  Eigen::Vector6d                         m_eLastJoinRobotInitPose;

};

#endif // STATEKEEPER_H
