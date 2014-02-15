#ifndef SIMROBOT_H
#define SIMROBOT_H

//#include <Network/Messages.h>
#include <URDFParser/TinyXMLTool.h>
#include <ModelGraph/ModelNode.h>
#include <Utils/ConvertName.h>


/*********************************************
  *
  * SIM ROBOT
  * SimRobot is an amalgamation of ModelNodes (Shapes, Constraints, and
  * RaycastVehicles) that make up a Robot. It has a unique name for the sum of
  * its parts, and also a 'Base' ModelNode that gives us a part from which to
  * measure a relative frame.
  * It also holds its URDF info, just in case we're through a server.
  *
  *********************************************/

class SimRobot
{
public:

  SimRobot(){
    m_bStateKeeperOn = false;
  }

  /////////////////////////////////

  void InitPoseOfBodyBaseWRTWorld(Eigen::Vector6d eRobotInitPoseInWorld){
    //TODO: Fix this to accept the difference.
//    eRobotInitPoseInWorld - m_Base->GetPose()
    m_Base->SetPose(eRobotInitPoseInWorld);
  }

  // set pose of body base in URDF as initial pose
  void InitPoseOfBodyBaseInURDF(){
    m_Base->SetPose(m_eRobotInitPoseInURDF);
  }

  /********************
   * SETTERS
   ********************/

  void SetName(std::string sName){
    m_sRobotName = sName;
  }

  void SetBase(ModelNode* Base){
    m_Base = Base;
  }

  void SetParts(std::vector<ModelNode*> vParts){
    m_vParts = vParts;
  }

  void SetProxyName(std::string sProxyName){
    m_sProxyName = sProxyName;
  }

  void SetRobotURDF(XMLDocument* pRobotURDF){
    m_pRobotURDF = pRobotURDF;
  }

  /********************
   * GETTERS
   ********************/

  string GetRobotName(){
    return m_sRobotName;
  }

  XMLDocument* GetRobotURDF(){
    return m_pRobotURDF;
  }

  std::vector<std::string> GetAllEntityNames(){
    std::vector<std::string> parts;
    for(unsigned int ii = 0; ii<m_vParts.size(); ii++){
      parts.push_back(m_vParts.at(ii)->GetName());
    }
    return parts;
  }

  // get the pose of robot base from model node
  Eigen::Vector6d GetRobotBasePose(){
    Eigen::Vector6d pose= m_Base->GetPose();
    return pose;
  }

//  // get name of all body that belong to the robot.
  vector<string> GetAllBodyName(){
    vector<string>  vBodyName;
    vector<string>  vAllBodyNameList = GetAllEntityNames();
    for(unsigned int i=0;i!=vAllBodyNameList.size();i++){
      string sFullName= vAllBodyNameList[i];
      if(GetRobotNameFromFullName(sFullName)== m_sRobotName ){
        vBodyName.push_back(sFullName);
      }
    }
    return vBodyName;
  }

  std::vector<ModelNode*> GetParts(){
    return m_vParts;
  }

  bool GetStateKeeperStatus(){
    return m_bStateKeeperOn;
  }

  /*****************************/

  // select between PD or PID control
  enum InitPoseMethod{
    FromXML = 1,
    FromServer = 2
  };

private:
  string                      m_sRobotName;
  string                      m_sProxyName;
  std::vector<ModelNode*>     m_vParts;
  bool                        m_bStateKeeperOn;
  ModelNode*                  m_Base;
  XMLDocument*                m_pRobotURDF;           // robot URDF file
  Eigen::Vector6d             m_eRobotInitPose;       // actual pose that init robot
  Eigen::Vector6d             m_eRobotInitPoseInURDF; // init pose in robot urdf file.
};

#endif
