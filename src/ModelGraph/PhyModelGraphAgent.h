#ifndef PHYMODELGRAPHAGENT_H
#define PHYMODELGRAPHAGENT_H

#include <ModelGraph/BulletModelGraphAgent.h>
// can also include other physci engine, e.g. ODE

class PhyModelGraphAgent
{

public:
    BulletModelGraphAgent  m_Agent;

    bool init()
    {
        if(m_Agent.Init()== true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};



#endif // BULLETMODELGRAPHAGENT_H
