#ifndef SIMVICON_H
#define SIMVICON_H


#include <Device/SimDevices.h>
#include <ModelGraph/PhyModelGraphAgent.h>

// return x,y,z,p,q,r of body A.
class SimVicon
{

public:
    // request bullet the pose of object.
    void init(string DeviceName,string BodyName, PhyModelGraphAgent& rPhyMGAgent)
    {
        m_rPhyMGAgent = rPhyMGAgent;
        m_sDeviceName  = DeviceName;
        m_sBodyName  = BodyName;
    }

    void Update()
    {
         // update pose
         m_rPhyMGAgent.m_Agent.GetEntity6Pose(m_sBodyName,m_Poses[0],m_Poses[1],m_Poses[2],m_Poses[3],m_Poses[4],m_Poses[5]);
    }

    Eigen::Vector6d GetLatestPose()
    {
        return m_Poses;
    }

    void PrintCurPose()
    {
        cout<<"[vicon] get pose x :"<<m_Poses[0]<<", y:"<<m_Poses[1]<<", z:"<<m_Poses[2]<<", p:"<<m_Poses[3]<<", q:"<<m_Poses[4]<<", r:"<<m_Poses[5]<<endl;
    }

private:
    PhyModelGraphAgent                 m_rPhyMGAgent;
    string                             m_sDeviceName;
    string                             m_sBodyName;
    Eigen::Vector6d                    m_Poses;

};


#endif // SIMVICON_H
