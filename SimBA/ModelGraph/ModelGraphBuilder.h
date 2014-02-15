#ifndef MODELGRAPHBUILDER_H
#define MODELGRAPHBUILDER_H

#include <ModelGraph/RenderEngine.h>
#include <ModelGraph/PhysicsEngine.h>
#include <SimRobots/SimRobot.h>
#include <SimRobots/SimWorld.h>

class ModelGraphBuilder{
public:

  enum BuildType{
    Physics = 1,
    Rendering = 2,
    All = 3
  };

  /////////////////////////////////////////
  /// FUNCTIONS
  /////////////////////////////////////////

  void AssociateWorldPhysics(SimWorld m_SimWorld){
    for(unsigned int ii=0; ii<m_SimWorld.m_WorldNodes.size(); ii++){
      ModelNode* part = m_SimWorld.m_WorldNodes.at(ii);
      if (dynamic_cast<Shape*>(part)){
        m_Phys.RegisterObject( part );
        Eigen::Vector6d ChildWorldPose;
        for(unsigned int ii = 0; ii < part->NumChildren(); ii++ ) {
          ChildWorldPose = _T2Cart(
                part->GetPoseMatrix() * part->m_vChildren.at(ii)->GetPoseMatrix());
          part->m_vChildren.at(ii)->SetPose(ChildWorldPose);
        }
      }
    }
  }

  void AssociatePhysicsShapes(SimRobot m_SimRobot){
    for(unsigned int ii=0; ii<m_SimRobot.GetParts().size(); ii++){
      ModelNode* part = m_SimRobot.GetParts().at(ii);
      if (dynamic_cast<Shape*>(part)){
        m_Phys.RegisterObject( part );
        Eigen::Vector6d ChildWorldPose;
        for(unsigned int ii = 0; ii < part->NumChildren(); ii++ ) {
          ChildWorldPose = _T2Cart(
                part->GetPoseMatrix() * part->m_vChildren.at(ii)->GetPoseMatrix());
          part->m_vChildren.at(ii)->SetPose(ChildWorldPose);
        }
      }
    }
  }

  void AssociatePhysicsConstraints(SimRobot m_SimRobot){
    for(unsigned int ii=0; ii<m_SimRobot.GetParts().size(); ii++){
      ModelNode* part = m_SimRobot.GetParts().at(ii);
      if (dynamic_cast<Constraint*>(part)){
        m_Phys.RegisterObject( part );
        Eigen::Vector6d ChildWorldPose;
        for(unsigned int ii = 0; ii < part->NumChildren(); ii++ ) {
          ChildWorldPose = _T2Cart(
                part->GetPoseMatrix() * part->m_vChildren.at(ii)->GetPoseMatrix());
          part->m_vChildren.at(ii)->SetPose(ChildWorldPose);
        }
      }
    }
  }

  void AssociatePhysicsVehicles(SimRobot m_SimRobot){
    for(unsigned int ii=0; ii<m_SimRobot.GetParts().size(); ii++){
      ModelNode* part = m_SimRobot.GetParts().at(ii);
      if (dynamic_cast<RaycastVehicle*>(part)){
        m_Phys.RegisterObject( part );
        Eigen::Vector6d ChildWorldPose;
        for(unsigned int ii = 0; ii < part->NumChildren(); ii++ ) {
          ChildWorldPose = _T2Cart(
                part->GetPoseMatrix() * part->m_vChildren.at(ii)->GetPoseMatrix());
          part->m_vChildren.at(ii)->SetPose(ChildWorldPose);
        }
      }
    }
  }

  void AssociateRobotPhysics(SimRobot m_SimRobot){
    AssociatePhysicsShapes(m_SimRobot);
    AssociatePhysicsVehicles(m_SimRobot);
    AssociatePhysicsConstraints(m_SimRobot);
  }

  /////////////////////////////////////////

  void RenderWorldGraph(SimWorld m_SimWorld){
    for(unsigned int ii=0; ii<m_SimWorld.m_WorldNodes.size(); ii++){
      ModelNode* part = m_SimWorld.m_WorldNodes.at(ii);
      m_Render.AddNode(part);
      Eigen::Vector6d ChildWorldPose;
      for (unsigned int ii = 0; ii < part->NumChildren(); ii++ ) {
        ChildWorldPose = _T2Cart(
              part->GetPoseMatrix() * part->m_vChildren[ii]->GetPoseMatrix());
        part->SetPose(ChildWorldPose);
      }
    }
  }

  void RenderRobotGraph(SimRobot m_SimRobot){
    for(unsigned int ii=0; ii<m_SimRobot.GetParts().size(); ii++){
      ModelNode* part = m_SimRobot.GetParts().at(ii);
      m_Render.AddNode(part);
      Eigen::Vector6d ChildWorldPose;
      for (unsigned int ii = 0; ii < part->NumChildren(); ii++ ) {
        ChildWorldPose = _T2Cart(
              part->GetPoseMatrix() * part->m_vChildren[ii]->GetPoseMatrix());
        part->SetPose(ChildWorldPose);
      }
    }
  }

  ////////////////////////////////////////

  void Init(int iBuildType, SimWorld& m_WorldModel,
            SimRobot& m_SimRobot, std::string sSimName){
    m_Phys.Init();
    m_Render.Init(sSimName);
    iBuildType = iBuildType;
    if(iBuildType == Physics){
      AssociateWorldPhysics(m_WorldModel);
      AssociateRobotPhysics(m_SimRobot);
    }
    else if(iBuildType == Rendering){
      RenderWorldGraph(m_WorldModel);
      RenderRobotGraph(m_SimRobot);
    }
    else if(iBuildType == All){
      AssociateWorldPhysics(m_WorldModel);
      AssociateRobotPhysics(m_SimRobot);
      RenderWorldGraph(m_WorldModel);
      RenderRobotGraph(m_SimRobot);
    }
    else{
      cout<<"[ModelGraphBuilder] Fatal Error! Unknown Build Type!"<<endl;
    }
  }

  void UpdateScene(bool bDebug = false){
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    if(bDebug == false){
      m_Phys.StepSimulation();
    }
    else{
      m_Phys.DebugDrawWorld();
    }
    m_Render.UpdateScene();
  }

  PhysicsEngine m_Phys;
  RenderEngine m_Render;

};

#endif // MODELGRAPHBUILDER_H
