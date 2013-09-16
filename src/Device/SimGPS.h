#ifndef SIMGPS_H
#define SIMGPS_H

#include <Device/SimDevices.h>
#include <ModelGraph/PhyModelGraphAgent.h>

// return x,y,z of a GPS sensor.
class SimGPS
{
public:
    void init(string DeviceName, PhyModelGraphAgent* mPhyMGAgent)
    {
        m_pPhysMGAgent = mPhyMGAgent;
        m_sDeviceName = DeviceName;
    }

    void Update()
    {
        m_pPhysMGAgent->m_Agent.GetEntity3Pose(m_sDeviceName,m_CurPose[0],m_CurPose[1],m_CurPose[2]);
    }

    void GetPose(Eigen::Vector3d &pose)
    {
        pose<<m_CurPose[0], m_CurPose[1],m_CurPose[2];
    }

    void PrintPose()
    {
        cout<<"[GPS PrintPose] current pose is x:"<<m_CurPose[0]<<" y:"<<m_CurPose[1]<<" z:"<<m_CurPose[2]<<endl;
    }

private:
    PhyModelGraphAgent*           m_pPhysMGAgent;
    Phys*                         m_pPhys;
    string                        m_sDeviceName;
    Eigen::Vector3d               m_CurPose;
};

#endif // SIMGPS_H
