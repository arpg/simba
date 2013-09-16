#ifndef STATEKEEPERURDFPARSER_H
#define STATEKEEPERURDFPARSER_H

#include <vector>
#include <Eigen/Eigen>
#include <URDFParser/TinXMLTool.h>

using namespace tinyxml2;
using namespace std;

namespace Eigen
{
    typedef Matrix<double, 6, 1> Vector6d;
}

// -----------------------------------------------------------------------------------------------------------------------
// parse world.xml file to get initial poses for StateKeeper
bool ParseWorldForInitialPoses(const char* filename, vector<Eigen::Vector6d>& vRobotInitPose)
{
    // make sure the vector is empty
    vRobotInitPose.clear();

    // open xml document
    XMLDocument doc;
    if(doc.LoadFile(filename) !=0)
    {
         printf("Cannot open %s\n", filename);
         return false;
    }

    XMLElement *pParent=doc.RootElement();
    XMLElement *pElement=pParent->FirstChildElement();

    // read high level parent (root parent)
    while (pElement)
    {
        const char* sRootContent = pElement->Name();

        if(strcmp(sRootContent,"robot")==0)
        {
            vector<double> vPose = GenNumFromChar(pElement->Attribute("pose"));
            Eigen::Vector6d ePose;
            ePose<<vPose[0], vPose[1], vPose[2], vPose[3], vPose[4], vPose[5];
            vRobotInitPose.push_back(ePose);
        }
        pElement=pElement->NextSiblingElement();
    }

    return true;
}



#endif // STATEKEEPERURDFPARSER_H
