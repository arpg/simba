#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "ModelGraph/ModelGraphBuilder.h"

class Controller
{
public:

  Controller(){
  }

  void init(string ControllerName, string sRobotName,
            string sProxyName, ModelGraphBuilder Scene){
    m_sControllerName = ControllerName;
    m_Scene = Scene;
    m_sProxyName = sProxyName;
    m_sRobotName = sRobotName;
  }

  string m_sControllerName;
  ModelGraphBuilder m_Scene;
  string m_sProxyName;
  string m_sRobotName;

};

#endif // CONTROLLER_H
