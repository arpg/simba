#ifndef URDF_PARSER_H
#define URDF_PARSER_H


#include <vector>
#include <Eigen/Eigen>
#include <URDFParser/TinyXMLTool.h>
#include <ModelGraph/Shape.h>
#include <ModelGraph/Constraint.h>
#include <ModelGraph/RaycastVehicle.h>
#include <SimDevices/SimDeviceInfo.h>
#include <Managers/WorldManager.h>


/////////////////////////////////
/// URDF PARSER
/// This file merges the RobotProxyURDFParser and the StateKeeperURDFParser
/// There was no reason that there should have been two files, as far as I can
/// tell. They parse the same files, after all.
/// ------------
/// Please notice that the URDF file here is not strictly the URDF format on the
/// internet. Our version is a bit different from the formal variety; We may
/// consider support for the strict URDF format in the future.
/////////////////////////////////

using namespace std;
namespace Eigen{
    typedef Matrix<double, 6, 1> Vector6d;
}

class URDF_Parser
{
public:
  URDF_Parser();

  // Parses the world for the mesh and conditions.
  bool ParseWorld(const char* filename, WorldManager& mWorldManager);

  // Parses the Robot.xml file describing a RaycastVehicle.
  bool ParseRaycastVehicle(XMLDocument* doc, Model& m_RobotModel,
                           Eigen::Vector6d& InitPose, string sProxyName);

  // ParseRobot really parses each of the robot parts, and then generates a set
  // of commands that the PhysicsEngine can use to create bullet objects.
  bool ParseRobot(XMLDocument* doc, Model& m_RobotModel,
                         Eigen::Vector6d& InitPose, string sProxyName);

  // ParseDevices uses the information given in the Robot.xml file to create the
  // sensor views that we see later in the Sim.
  bool ParseDevices(XMLDocument& doc,
                          vector<SimDeviceInfo>& m_vSimDeviceInfo,
                          string sProxyName);

  // This method is used in StateKeeper to initialize the position of every
  // object in the RobotProxy.
  bool ParseWorldForInitialPoses(const char* filename,
                                 vector<Eigen::Vector6d>& vRobotInitPose);

};

#endif // URDF_PARSER_H
