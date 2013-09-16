#ifndef WORLDMANAGER_H
#define WORLDMANAGER_H

#include <vector>
#include <string>
#include <iostream>
#include <URDFParser/RobotProxyURDFParser.h>
using namespace std;

class WorldManager
{
public:
    vector<double> vWorldPose;
    vector<double> vRobotPose;
    vector<double> vLightPose;
    std::string    m_sMesh;
    int            iScale;
    int            iMass;


    void PrintAll()
    {
        cout<<"---------------------------------------------------------------------------"<<endl<<
              "finish build world from world.XML"<<endl<<
              "mesh dir is "<<m_sMesh<<endl<<"Scale is "<<iScale<<". mass is "<<iMass<<endl<<
              "world pose is "<<vWorldPose[0]<<", "<<vWorldPose[1]<<", "<<vWorldPose[2]<<", "<<endl<<
              "robot pose is "<<vRobotPose[0]<<", "<<vRobotPose[1]<<", "<<vRobotPose[2]<<", "<<vRobotPose[3]<<", "<<vRobotPose[4]<<", "<<vRobotPose[5]<<endl<<
              "light pose is "<<vLightPose[0]<<", "<<vLightPose[1]<<", "<<vLightPose[2]<<", "<<endl<<
              "---------------------------------------------------------------------------"<<endl;
    }

};

#endif
