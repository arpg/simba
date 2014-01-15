#ifndef CARCONTROLLER_H
#define CARCONTROLLER_H

class CarController
{
//public:
//    // -----------------------------------------------------------------------------------------------------------------------
//    void init(string sRobotName, string sProxyName, PhyModelGraphAgent& rPhyMGAgent)
//    {
////        m_sControllerName = ControllerName; this may be necessary later. but by now every robot only have one controller.
//        m_rPhyMGAgent = rPhyMGAgent;
//        m_sProxyName = sProxyName;
//        m_sRobotName = sRobotName;
//        m_eCommand<<0,0,0,0,0,0;
//    }


//    // -----------------------------------------------------------------------------------------------------------------------
//    // update command from statekeeper (in 'with statekeeper' mode) or node controller (in 'with Network' mode) or robot proxy (in 'without network' mode)
//    void UpdateCommand(vector<string> vBodyFullName, Eigen::Vector6d eCommand)
//    {
//        m_eCommand = eCommand;
//        m_vBodyFullName = vBodyFullName;
//        if(eCommand[0]!=0 || eCommand[1]!=0 || eCommand[2]!=0 || eCommand[3]!=0 ||eCommand[4]!=0 || eCommand[5]!=0)
//        {
//            m_bCommandIsLatestFlag = true;
//        }
//        else
//        {
//            m_bCommandIsLatestFlag = false;
//        }
//    }


//    // -----------------------------------------------------------------------------------------------------------------------
//    // apply command in both steering and troque. After we apply one command, we set the m_bCommandIsLatestFlag to false as old one
//    bool ApplyCommand()
//    {
//        // improve: apply in model, which can be bicycle model and uni-cycle model
//        // m_MobileModel.BicycleModel(m_SimRobot, fDesireVelocity, fDesireSteering);

//        // apply torque,
//        if(m_eCommand[0]!=0 || m_eCommand[1]!=0 || m_eCommand[2]!=0)
//        {
//            Eigen::Vector3d eTorque;
//            eTorque<<m_eCommand[0],m_eCommand[1],m_eCommand[2];
//            for(int i=0;i!=m_vBodyFullName.size();i++)
//            {
//                string sBodyFullName = m_vBodyFullName[i];
//                m_rPhyMGAgent.m_Agent.ApplyTorque(sBodyFullName, eTorque);
//            }
//        }

//        // apply steering
//        if(m_eCommand[3]!=0 || m_eCommand[4]!=0 || m_eCommand[5]!=0)
//        {
//            Eigen::Vector3d eSteering;
//            eSteering<<m_eCommand[3],m_eCommand[4],m_eCommand[5];
//            for(int i=0;i!=m_vBodyFullName.size();i++)
//            {
//                string sBodyFullName = m_vBodyFullName[i];
//                m_rPhyMGAgent.m_Agent.ApplySteering(sBodyFullName, eSteering);
//            }
//        }

//        // after we apply command, set flag to false
//        m_bCommandIsLatestFlag = false;
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
//            eCommand = m_eCommand;
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
//        m_rPhyMGAgent.m_Agent.ApplyTorque(sFullName,eTorque);
//    }



//    //-----------------------------------------------------------------------------------------------
//    void ApplySteering(string sBodyFirstName, Eigen::Vector3d eDegree)
//    {
//        string sFullName = sBodyFirstName+"@"+m_sRobotName;
//        m_rPhyMGAgent.m_Agent.ApplySteering(sFullName,eDegree);
//    }


//    //-----------------------------------------------------------------------------------------------
//    // set rotation basis matrix
//    void SetBasis(string sBodyFirstName, Eigen::Matrix3d eBasis)
//    {
//        string sFullName = sBodyFirstName+"@"+m_sRobotName;

//    }


//    //-----------------------------------------------------------------------------------------------
//    void SetDesiredAnlge( double dAngle )
//    {

//    }


//private:
//    bool                           m_bCommandIsLatestFlag;  // mark if command is latest. if not, we will not send this command to statekeeper.
//    string                         m_sControllerName;
//    string                         m_sRobotName;
//    string                         m_sProxyName;
//    vector<string>                 m_vBodyFullName;    // body that will be applied latest command. In many Cases it will be pair body.
//    Eigen::Vector6d                m_eCommand;         // latest command
//    PhyModelGraphAgent             m_rPhyMGAgent;
};

#endif // CARCONTROLLER_H
