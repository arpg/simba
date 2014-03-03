#ifndef SIMPLECONTROLLER_H
#define SIMPLECONTROLLER_H

#include <SimDevices/SimDeviceInfo.h>
#include "PB_Headers/SimMessages.pb.h"

class SimpleController: public SimDeviceInfo
{
public:

  SimpleController(string sDeviceName, string sRobotName){
    SetDeviceName(sDeviceName);
    SetRobotName(sRobotName);
  }

  ///////////////////////////////////////////////////

  void UpdateCommand(PoseMsg& Command){
    Eigen::Vector6d eCommand;
    eCommand<<Command.x(), Command.y(), Command.z(),
        Command.p(),Command.q(),Command.r();
    vector<string> vBodyFullName;
    for(int j =0; j!= Command.body_name_size();j++){
      string sBodyFullName = Command.body_name(j);
      vBodyFullName.push_back(sBodyFullName);
    }
    m_eCommand.push_back(eCommand);
    m_vBodyFullName = vBodyFullName;
    if(eCommand[0]!=0 || eCommand[1]!=0 || eCommand[2]!=0 || eCommand[3]!=0 ||
       eCommand[4]!=0 || eCommand[5]!=0){
      m_bCommandIsLatestFlag = true;
    }
    else{
      m_bCommandIsLatestFlag = false;
    }
  }

  // mark if command is latest. if not, we will not send this command to statekeeper.
  bool                           m_bCommandIsLatestFlag;
  // body that will be applied latest command. In many Cases it will be pair body.
  vector<string>                 m_vBodyFullName;
  std::vector< Eigen::Vector6d > m_eCommand;

};

#endif // SIMROBOTCONTROLLER_H
