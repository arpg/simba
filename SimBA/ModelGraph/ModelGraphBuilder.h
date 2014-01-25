#ifndef MODELGRAPHBUILDER_H
#define MODELGRAPHBUILDER_H

#include <ModelGraph/RenderEngine.h>
#include <ModelGraph/PhysicsEngine.h>

class ModelGraphBuilder{
public:

  enum BuildType{
    Physics = 1,
    Rendering = 2,
    All = 3
  };


  ModelGraphBuilder(){
  }

  /////////////////////////////////////////
  /// FUNCTIONS
  /////////////////////////////////////////

  void AssociatePhysicsShapes(RobotModel* RobotModel){
    for(unsigned int ii=0; ii<RobotModel->m_vParts.size(); ii++){
      ModelNode* part = RobotModel->m_vParts.at(ii);
      if (dynamic_cast<Shape*>(part)){
        m_Phys.RegisterObject( part );
        Eigen::Vector6d ChildWorldPose;
        for(unsigned int ii = 0; ii < part->NumChildren(); ii++ ) {
          ChildWorldPose = _T2Cart(
                part->GetPoseMatrix() * part->m_vChildren.at(ii)->GetPoseMatrix());
          part->m_vChildren.at(ii)->SetPose(ChildWorldPose);
          //          AssociatePhysicsShapes(m_Phys, *part->m_vChildren.at(ii));
        }
      }
    }
  }

  void AssociatePhysicsConstraints(RobotModel* RobotModel){
    for(unsigned int ii=0; ii<RobotModel->m_vParts.size(); ii++){
      ModelNode* part = RobotModel->m_vParts.at(ii);
      if (dynamic_cast<Constraint*>(part)){
        m_Phys.RegisterObject( part );
        Eigen::Vector6d ChildWorldPose;
        for(unsigned int ii = 0; ii < part->NumChildren(); ii++ ) {
          ChildWorldPose = _T2Cart(
                part->GetPoseMatrix() * part->m_vChildren.at(ii)->GetPoseMatrix());
          part->m_vChildren.at(ii)->SetPose(ChildWorldPose);
          //          AssociatePhysicsConstraints(m_Phys, *part.m_vChildren.at(ii));
        }
      }
    }
  }

  void AssociatePhysicsVehicles(RobotModel* RobotModel){
    for(unsigned int ii=0; ii<RobotModel->m_vParts.size(); ii++){
      ModelNode* part = RobotModel->m_vParts.at(ii);
      if (dynamic_cast<RaycastVehicle*>(part)){
        m_Phys.RegisterObject( part );
        Eigen::Vector6d ChildWorldPose;
        for(unsigned int ii = 0; ii < part->NumChildren(); ii++ ) {
          ChildWorldPose = _T2Cart(
                part->GetPoseMatrix() * part->m_vChildren.at(ii)->GetPoseMatrix());
          part->m_vChildren.at(ii)->SetPose(ChildWorldPose);
          //        AssociatePhysicsVehicles(m_Phys, *part.m_vChildren.at(ii));
        }
      }
    }
  }

  void AssociatePhysics(RobotModel* m_RobotModel){
    AssociatePhysicsShapes(m_RobotModel);
    AssociatePhysicsVehicles(m_RobotModel);
    AssociatePhysicsConstraints(m_RobotModel);
  }


  /////////////////////////////////////////

  void RenderGraph(RobotModel* m_RobotModel){
    for(unsigned int ii=0; ii<m_RobotModel->m_vParts.size(); ii++){
      ModelNode* part = m_RobotModel->m_vParts.at(ii);
      m_Render.AddNode(part);
      Eigen::Vector6d ChildWorldPose;
      for (unsigned int ii = 0; ii < part->NumChildren(); ii++ ) {
        ChildWorldPose = _T2Cart(
              part->GetPoseMatrix() * part->m_vChildren[ii]->GetPoseMatrix());
        part->SetPose(ChildWorldPose);
//        RenderGraph(m_Render, *(part.m_vChildren[ii]));
      }
    }
  }

  ////////////////////////////////////////

  void Init(int iBuildType, RobotModel* m_RobotModel, std::string sSimName){
    m_Phys.Init();
    m_Render.Init(sSimName);
    iBuildType = iBuildType;
    if(iBuildType == Physics){
      AssociatePhysics(m_RobotModel);
    }
    else if(iBuildType == Rendering){
      RenderGraph(m_RobotModel);
    }
    else if(iBuildType == All){
      AssociatePhysics(m_RobotModel);
      RenderGraph(m_RobotModel);
    }
    else{
      cout<<"[ModelGraphBuilder] Fatal Error! Unknown Build Type!"<<endl;
    }
  }

  void UpdateScene(){
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    m_Phys.DebugDrawWorld();
    m_Phys.StepSimulation();
    m_Render.UpdateScene();
  }

  PhysicsEngine m_Phys;
  Render m_Render;

};

#endif // MODELGRAPHBUILDER_H
