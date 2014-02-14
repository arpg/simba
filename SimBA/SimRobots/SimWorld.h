#ifndef SIMWORLD_H
#define SIMWORLD_H

#include <vector>
#include <string>
#include <iostream>
#include <ModelGraph/ModelNode.h>

using namespace std;

class SimWorld
{

public:

    vector<double>          m_vWorldPose;
    vector<double>          m_vRobotPose;
    std::vector<ModelNode*> m_WorldNodes;
    std::string             m_sMesh;


    void PrintAll(){
        cout<<"--------------------------------------------------------"<<endl<<
              "Finished building world from world.XML"<<endl<<
              "--------------------------------------------------------"<<endl;
    }



};

#endif
