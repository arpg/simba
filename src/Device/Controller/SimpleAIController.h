// provide a simple AI that control Robot in RobotProxy

#ifndef SIMPLEAICONTROLLER_H
#define SIMPLEAICONTROLLER_H

#include <ModelGraph/PhyModelGraphAgent.h>

class SimpleAIController
{
public:
    void init(string DeviceName, PhyModelGraphAgent& rPhyMGAgent)
    {
        m_sDeviceName = DeviceName;
        m_rPhyMGAgent = rPhyMGAgent;
    }

    // define some AI behaviour here.



private:
    string                         m_sDeviceName;
    PhyModelGraphAgent             m_rPhyMGAgent;
};



#endif // SIMPLEAICONTROLLER_H
