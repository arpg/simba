#ifndef SIMPLECONTROLLER_H
#define SIMPLECONTROLLER_H

#include <SimDevices/Controller/Controller.h>

class SimpleController: public Controller
{
public:

    SimpleController(string sDeviceName, string sRobotName, string sProxyName){
      SetControllerName(sDeviceName);
      SetRobotName(sRobotName);
      SetProxyName(sProxyName);
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


//    // -----------------------------------------------------------------------------------------------------------------------
//    // apply command in both steering and troque. After we apply one command, we set the m_bCommandIsLatestFlag to false as old one
//    bool ApplyCommand()
//    {
//        // improve: apply in model, which can be bicycle model and uni-cycle model
//        // m_MobileModel.BicycleModel(m_SimRobot, fDesireVelocity, fDesireSteering);

//        // apply torque,

//        for (unsigned int ii = 0; ii < m_eCommand.size(); ii++) {
//            if(m_eCommand[ii][0]!=0 || m_eCommand[ii][1]!=0 || m_eCommand[ii][2]!=0)
//            {
//                Eigen::Vector3d eTorque;
//                eTorque[0] = m_eCommand[ii][0];
//                eTorque[1] = m_eCommand[ii][1];
//                eTorque[2] = m_eCommand[ii][2];
//                string sBodyFullName = m_vBodyFullName[ii];
//                m_rPhysWrapper.ApplyTorque(sBodyFullName, eTorque);
//            }
//        }

//        for (unsigned int ii = 0; ii < m_eCommand.size(); ii++) {
//            if(m_eCommand[ii][3]!=0 || m_eCommand[ii][4]!=0 || m_eCommand[ii][5]!=0)
//            {
//                Eigen::Vector3d eSteering;
//                eSteering[0] = m_eCommand[ii][3];
//                eSteering[1] = m_eCommand[ii][4];
//                eSteering[2] = m_eCommand[ii][5];
//                string sBodyFullName = m_vBodyFullName[ii];
//                m_rPhysWrapper.ApplySteering(sBodyFullName, eSteering);
//            }
//        }

//        // after we apply command, set flag to false
//        m_bCommandIsLatestFlag = false;
//        m_eCommand.clear();
//        m_vBodyFullName.clear();
//        return true;
//    }


//    // -----------------------------------------------------------------------------------------------------------------------
//    // return latest command and body will apply such command for state keeper to publish
//    // not using this function if we do not sync command
//    void GetLatestCommandAndBodyFullName(vector<string>& vBodysFullName, Eigen::Vector6d& eCommand)
//    {
//        if(m_bCommandIsLatestFlag==true)
//        {
//            vBodysFullName = m_vBodyFullName;
//            eCommand = m_eCommand[m_eCommand.size() - 1];
//            m_bCommandIsLatestFlag = false;
//        }
//        else
//        {
//            vBodysFullName = m_vBodyFullName;
//            eCommand<<0,0,0,0,0,0;
//        }
//    }


//protected:
//    //-----------------------------------------------------------------------------------------------
//    void ApplyTorque(string sBodyFirstName, Eigen::Vector3d eTorque)
//    {
//        string sFullName = sBodyFirstName+"@"+m_sRobotName;
//        cout<<"apply torque to body "<<sFullName<<endl;
//        m_rPhysWrapper.ApplyTorque(sFullName,eTorque);
//    }


//    //-----------------------------------------------------------------------------------------------
//    void ApplySteering(string sBodyFirstName, Eigen::Vector3d eDegree)
//    {
//        string sFullName = sBodyFirstName+"@"+m_sRobotName;
//        m_rPhysWrapper.ApplySteering(sFullName,eDegree);
//    }


    bool                           m_bCommandIsLatestFlag;  // mark if command is latest. if not, we will not send this command to statekeeper.
    vector<string>                 m_vBodyFullName;    // body that will be applied latest command. In many Cases it will be pair body.
    std::vector< Eigen::Vector6d > m_eCommand;         // latest command

};

#endif // SIMROBOTCONTROLLER_H
