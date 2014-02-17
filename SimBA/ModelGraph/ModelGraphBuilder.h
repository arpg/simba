#ifndef MODELGRAPHBUILDER_H
#define MODELGRAPHBUILDER_H

#include <ModelGraph/RenderEngine.h>
#include <ModelGraph/PhysicsEngine.h>
#include <SimRobots/SimRobot.h>
#include <SimRobots/SimWorld.h>

class ModelGraphBuilder{
public:

  /////////////////////////////////////////
  /// FUNCTIONS
  /////////////////////////////////////////

  void AssociateWorldPhysics(SimWorld m_SimWorld){
    for(unsigned int ii=0; ii<m_SimWorld.m_WorldNodes.size(); ii++){
      ModelNode* part = m_SimWorld.m_WorldNodes.at(ii);
      if (dynamic_cast<Shape*>(part)){
        m_Phys.RegisterObject( part );
      }
    }
  }

  void AssociatePhysicsShapes(SimRobot& m_SimRobot){
    for(unsigned int ii=0; ii<m_SimRobot.GetParts().size(); ii++){
      ModelNode* part = m_SimRobot.GetParts().at(ii);
      if (dynamic_cast<Shape*>(part)){
        Eigen::Vector6d newPose = m_PoseRW + part->GetPose();
        part->SetPose( newPose );
        m_Phys.RegisterObject( part );
      }
    }
  }

  void AssociatePhysicsConstraints(SimRobot& m_SimRobot){
    for(unsigned int ii=0; ii<m_SimRobot.GetParts().size(); ii++){
      ModelNode* part = m_SimRobot.GetParts().at(ii);
      // Constraints are in their own reference frame, so they
      // don't need modification.
      if (dynamic_cast<Constraint*>(part)){
        m_Phys.RegisterObject( part );
      }
    }
  }

  void AssociatePhysicsVehicles(SimRobot& m_SimRobot){
    for(unsigned int ii=0; ii<m_SimRobot.GetParts().size(); ii++){
      ModelNode* part = m_SimRobot.GetParts().at(ii);
      if (dynamic_cast<RaycastVehicle*>(part)){
        Eigen::Vector6d newPose = m_PoseRW + part->GetPose();
        part->SetPose( newPose );
        m_Phys.RegisterObject( part );
      }
    }
  }

  void AssociateRobotPhysics(SimRobot& m_SimRobot){
    AssociatePhysicsShapes(m_SimRobot);
    AssociatePhysicsVehicles(m_SimRobot);
    AssociatePhysicsConstraints(m_SimRobot);
  }

  /////////////////////////////////////////

  void RenderWorldGraph(SimWorld m_SimWorld){
    for(unsigned int ii=0; ii<m_SimWorld.m_WorldNodes.size(); ii++){
      ModelNode* part = m_SimWorld.m_WorldNodes.at(ii);
      m_Render.AddNode(part);
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

  void Init(SimWorld& m_WorldModel,
            SimRobot& m_SimRobot, std::string sSimName){
    m_Phys.Init();
    m_Render.Init(sSimName);
    if(m_SimRobot.GetStateKeeperStatus()==true){
      // Get the PoseRW from the StateKeeper, and not the World.XML file.
    }
    else{
      m_PoseRW<<m_WorldModel.m_vRobotPose[0], m_WorldModel.m_vRobotPose[1],
          m_WorldModel.m_vRobotPose[2], m_WorldModel.m_vRobotPose[3],
          m_WorldModel.m_vRobotPose[4], m_WorldModel.m_vRobotPose[5];
    }
    // We move the parts around to their respective places in
    // Associate****Physics; RenderWorldGraph should keep the same places.
    // This inheritance pattern is what makes the ModelNode class so important.
    AssociateWorldPhysics(m_WorldModel);
    AssociateRobotPhysics(m_SimRobot);
    RenderWorldGraph(m_WorldModel);
    RenderRobotGraph(m_SimRobot);
    m_Render.AddToScene();
    m_Render.CompleteScene();
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

  Eigen::Vector6d m_PoseRW;
  PhysicsEngine m_Phys;
  RenderEngine m_Render;

};

#endif // MODELGRAPHBUILDER_H
