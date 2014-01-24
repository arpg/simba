#ifndef RAYCASTVEHICLE_H
#define RAYCASTVEHICLE_H

#include <ModelGraph/ModelNode.h>

// It's a simple class, but enough to hold the info we need.

class RaycastVehicle : public ModelNode
{
public:

  RaycastVehicle(std::string sName, double* dParameters, Eigen::Vector6d dPose){
    m_dParameters = dParameters;
    m_dPose = dPose;
    m_sName = sName;
  }

  double* GetParameters(){
    return m_dParameters;
  }

  double* m_dParameters;

};



#endif // RAYCASTVEHICLE_H
