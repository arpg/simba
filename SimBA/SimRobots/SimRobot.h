#ifndef SIMROBOT_H
#define SIMROBOT_H

//#include <Network/Messages.h>
#include <ModelGraph/ModelGraphBuilder.h>
#include <URDFParser/URDF_Parser.h>
#include <SimDevices/Controller/Controller.h>
#include <Utils/ConvertName.h>


// Sim Robot (can works with PID control and bicycle & uni-cycle models).
// This can be any kind of robot, a Car, Pan-Tilt...
// This can be main robot (my robot) or other player's robot.
class SimRobot
{
public:

  SimRobot(){

  }

  // select between PD or PID control
  enum InitPoseMethod{
    FromXML = 1,
    FromServer = 2
  };

  // init mesh (appearance) of our robot. The initial pose of the robot is decided in the following two ways
  // 1, if proxy runs in non-StateKeeper mode, the initial pose of robot is readed from its's urdf file.
  // 2, if proxy runs in on-StateKeeper mode, the initial pose of robot is readed from StateKeeper.
  ~SimRobot()
  {
//    // 1, delete all constraints
//    // 1.1, delete all hinge constriants
//    std::map<string, btHingeConstraint*>::iterator Hiter = m_rPhysWrapper.m_mHinge.begin();
//    for(; Hiter!=m_rPhysWrapper.m_mHinge.end(); Hiter++)
//    {
//      string sFullHingeConstraintName = Hiter->first;
//      int index = sFullHingeConstraintName.find("@");
//      if(index!=-1)
//      {
//        string substr = sFullHingeConstraintName.substr(index+1, sFullHingeConstraintName.size()-index);

//        if(substr == m_sRobotName)
//        {
//          cout<<"delete "<<sFullHingeConstraintName<<endl;
//          m_rPhysWrapper.DeleteHingeConstraintFromDynamicsWorld(sFullHingeConstraintName);
//          m_rPhysWrapper.m_mHinge.erase(Hiter);
//        }
//      }
//    }


//    // 1.2, delete all hinge2 constraints
//    std::map<string, btHinge2Constraint*>::iterator H2iter = m_rPhysWrapper.m_mHinge2.begin();
//    for(; H2iter!=m_rPhysWrapper.m_mHinge2.end(); H2iter++)
//    {
//      string sFullHinge2ConstraintName = H2iter->first;
//      int index = sFullHinge2ConstraintName.find("@");
//      if(index!=-1)
//      {
//        string substr = sFullHinge2ConstraintName.substr(index+1, sFullHinge2ConstraintName.size()-index);

//        if(substr == m_sRobotName)
//        {
//          cout<<"delete "<<sFullHinge2ConstraintName<<endl;
//          m_rPhysWrapper.DeleteHinge2ConstraintFromDynamicsWorld(sFullHinge2ConstraintName);
//          m_rPhysWrapper.m_mHinge2.erase(H2iter);
//        }
//      }
//    }


//    // 2, delete all body and entity
//    std::map<string, boost::shared_ptr<Entity> >::iterator iter =m_rPhysWrapper.m_mShapes.begin();
//    for(;iter!=m_rPhysWrapper.m_mShapes.end(); iter++)
//    {
//      string sFullName = iter->first;
//      int index = sFullName.find("@");
//      if(index!=-1)
//      {
//        string substr = sFullName.substr(index+1, sFullName.size()-index);

//        if(substr == m_sRobotName)
//        {
//          cout<<"try to delete "<<sFullName<<endl;
//          m_rPhysWrapper.DeleteRigidBodyFromDynamicsWorld(sFullName);
//          //                     m_rPhysWrapper.m_mShapes.erase(iter); //**** need to delete it. implement it in the furture ****
//        }
//      }
//    }
  }

  bool Init(string ProxyName, bool bStateKeeperOn, XMLDocument& doc){
    URDF_Parser m_Parser;
    m_sProxyName = ProxyName;
    m_pRobotURDF = doc.ToDocument();
    m_Parser.ParseRobot(m_pRobotURDF->GetDocument(), m_RobotModel,
                        m_sProxyName);
    m_sRobotName = m_RobotModel.GetName();
//    // If we run in 'without Network mode', we need to init robot pose ourselves. or this pose will be read from server.
//    // and we need to build the ModelGraph manaully once we get the pose from statekeeper
//    if(bStateKeeperOn == false){
//      Eigen::Vector6d RobotInitPoseInWorld;
//      RobotInitPoseInWorld<<0,0,-2,0,0,1.57;
//      InitPoseOfBodyBaseWRTWorld(RobotInitPoseInWorld);
//    }
    cout<<"[SimRobot] Build robot "<<m_sRobotName<<" success."<<endl;
    return true;
  }

  void InitPoseOfBodyBaseWRTWorld(Eigen::Vector6d eRobotInitPoseInWorld){
    m_RobotModel.m_Base->GetPose() =
        eRobotInitPoseInWorld - m_RobotModel.m_Base->GetPose();
  }

  // set pose of body base in URDF as initial pose
  void InitPoseOfBodyBaseInURDF(){
    m_RobotModel.m_Base->GetPose() = m_eRobotInitPoseInURDF;
  }

  // get the pose of robot base from model node
  Eigen::Vector6d GetRobotBasePose(){
    ModelNode* pNode= m_RobotModel.m_Base;
    Eigen::Vector6d pose= pNode->GetPose();
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

  /********************
   * GETTERS
   ********************/

  string GetRobotName(){
    return m_sRobotName;
  }

  XMLDocument* GetRobotURDF(){
    return m_pRobotURDF;
  }

  RobotModel* GetRobotModel(){
    return &m_RobotModel;
  }

  std::vector<std::string> GetAllEntityNames(){
    std::vector<std::string> parts;
    for(unsigned int ii = 0; ii<m_RobotModel.m_vParts.size(); ii++){
      parts.push_back(m_RobotModel.m_vParts.at(ii)->GetName());
    }
    return parts;
  }

  /*****************************/

private:
  string                      m_sRobotName;
  string                      m_sProxyName;
  RobotModel                  m_RobotModel;           // model graph
  XMLDocument*                m_pRobotURDF;           // robot URDF file
  Eigen::Vector6d             m_eRobotInitPose;       // actual pose that init robot
  Eigen::Vector6d             m_eRobotInitPoseInURDF; // init pose in robot urdf file.
};

#endif
