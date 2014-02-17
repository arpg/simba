#ifndef RAYCASTVEHICLE_H
#define RAYCASTVEHICLE_H

#include <ModelGraph/ModelNode.h>
#include <ModelGraph/VehicleEnums.h>

// It's a simple class, but enough to hold the info we need.

class RaycastVehicle : public ModelNode
{
public:

  RaycastVehicle(std::string sName, vector<double> dParameters, Eigen::Vector6d dPose){
    m_dParameters = dParameters;
    m_dPose = dPose;
    m_sName = sName;
  }

  vector<double> GetParameters(){
    return m_dParameters;
  }


  vector<double> m_dParameters;

};



#endif // RAYCASTVEHICLE_H
