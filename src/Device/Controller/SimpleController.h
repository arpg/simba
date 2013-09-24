#ifndef SIMPLECONTROLLER_H
#define SIMPLECONTROLLER_H

#include <Device/SimDevices.h>
#include <ModelGraph/PhyModelGraphAgent.h>
#include <Eigen/Eigen>



class SimpleController
{
public:
    string                         m_sDeviceName;


    // -----------------------------------------------------------------------------------------------------------------------
    void init(string sDeviceName, string sRobotName, string sProxyName, PhyModelGraphAgent& rPhyMGAgent)
    {
        m_sDeviceName = sDeviceName;
        m_rPhyMGAgent = rPhyMGAgent;
        m_sProxyName  = sProxyName;
        m_sRobotName  = sRobotName;
//        Eigen::Vector6d temp;
//        temp << 0, 0, 0, 0, 0, 0;
//        m_eCommand.push_back( temp );
    }


    // -----------------------------------------------------------------------------------------------------------------------
    // update command from statekeeper (in 'with statekeeper' mode) or node controller (in 'with Network' mode) or robot proxy (in 'without network' mode)
    void UpdateCommand(vector<string> vBodyFullName, Eigen::Vector6d eCommand)
    {
        m_eCommand.push_back(eCommand);
        m_vBodyFullName = vBodyFullName;
        if(eCommand[0]!=0 || eCommand[1]!=0 || eCommand[2]!=0 || eCommand[3]!=0 ||eCommand[4]!=0 || eCommand[5]!=0)
        {
            m_bCommandIsLatestFlag = true;
        }
        else
        {
            m_bCommandIsLatestFlag = false;
        }
    }


    // -----------------------------------------------------------------------------------------------------------------------
    // apply command in both steering and troque. After we apply one command, we set the m_bCommandIsLatestFlag to false as old one
    bool ApplyCommand()
    {
        // improve: apply in model, which can be bicycle model and uni-cycle model
        // m_MobileModel.BicycleModel(m_SimRobot, fDesireVelocity, fDesireSteering);

        // apply torque,

        for (int ii = 0; ii < m_eCommand.size(); ii++) {
            if(m_eCommand[ii][0]!=0 || m_eCommand[ii][1]!=0 || m_eCommand[ii][2]!=0)
            {
                Eigen::Vector3d eTorque;
                eTorque[0] = m_eCommand[ii][0];
                eTorque[1] = m_eCommand[ii][1];
                eTorque[2] = m_eCommand[ii][2];
                string sBodyFullName = m_vBodyFullName[ii];
                m_rPhyMGAgent.m_Agent.ApplyTorque(sBodyFullName, eTorque);
            }
        }

        for (int ii = 0; ii < m_eCommand.size(); ii++) {
            if(m_eCommand[ii][3]!=0 || m_eCommand[ii][4]!=0 || m_eCommand[ii][5]!=0)
            {
                Eigen::Vector3d eSteering;
                eSteering[0] = m_eCommand[ii][3];
                eSteering[1] = m_eCommand[ii][4];
                eSteering[2] = m_eCommand[ii][5];
                string sBodyFullName = m_vBodyFullName[ii];
                m_rPhyMGAgent.m_Agent.ApplySteering(sBodyFullName, eSteering);
            }
        }


        // apply steering
//        if(m_eCommand[3]!=0 || m_eCommand[4]!=0 || m_eCommand[5]!=0)
//        {
//            Eigen::Vector3d eSteering;
//            eSteering<<m_eCommand[3],m_eCommand[4],m_eCommand[5];
//            for(unsigned int i=0;i!=m_vBodyFullName.size();i++)
//            {
//                string sBodyFullName = m_vBodyFullName[i];
//                m_rPhyMGAgent.m_Agent.ApplySteering(sBodyFullName, eSteering);
//            }
//        }

        // after we apply command, set flag to false
        m_bCommandIsLatestFlag = false;
        m_eCommand.clear();
        m_vBodyFullName.clear();
        return true;
    }


    // -----------------------------------------------------------------------------------------------------------------------
    // return latest command and body will apply such command for state keeper to publish
    // not using this function if we do not sync command
    void GetLatestCommandAndBodyFullName(vector<string>& vBodysFullName, Eigen::Vector6d& eCommand)
    {
        if(m_bCommandIsLatestFlag==true)
        {
            vBodysFullName = m_vBodyFullName;
            eCommand = m_eCommand[m_eCommand.size() - 1];
            m_bCommandIsLatestFlag = false;
        }
        else
        {
            vBodysFullName = m_vBodyFullName;
            eCommand<<0,0,0,0,0,0;
        }
    }


protected:
    //-----------------------------------------------------------------------------------------------
    void ApplyTorque(string sBodyFirstName, Eigen::Vector3d eTorque)
    {
        string sFullName = sBodyFirstName+"@"+m_sRobotName;
        cout<<"apply torque to body "<<sFullName<<endl;
        m_rPhyMGAgent.m_Agent.ApplyTorque(sFullName,eTorque);
    }


    //-----------------------------------------------------------------------------------------------
    void ApplySteering(string sBodyFirstName, Eigen::Vector3d eDegree)
    {
        string sFullName = sBodyFirstName+"@"+m_sRobotName;
        m_rPhyMGAgent.m_Agent.ApplySteering(sFullName,eDegree);
    }




private:
    bool                           m_bCommandIsLatestFlag;  // mark if command is latest. if not, we will not send this command to statekeeper.
    string                         m_sRobotName;
    string                         m_sProxyName;
    vector<string>                 m_vBodyFullName;    // body that will be applied latest command. In many Cases it will be pair body.
    std::vector< Eigen::Vector6d > m_eCommand;         // latest command
    PhyModelGraphAgent             m_rPhyMGAgent;
};

#endif // SIMROBOTCONTROLLER_H
