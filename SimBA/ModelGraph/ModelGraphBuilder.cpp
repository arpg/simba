#include <ModelGraph/ModelGraphBuilder.h>

/////////////////////////////////////////
/// FUNCTIONS
/////////////////////////////////////////

void ModelGraphBuilder::AssociateWorldPhysics(SimWorld m_SimWorld){
  for(unsigned int ii=0; ii<m_SimWorld.m_WorldNodes.size(); ii++){
    ModelNode* part = m_SimWorld.m_WorldNodes.at(ii);
    if (dynamic_cast<Shape*>(part)){
      m_Phys.RegisterObject( part );
    }
  }
}

void ModelGraphBuilder::AssociatePhysicsShapes(SimRobot& m_SimRobot){
  for(unsigned int ii=0; ii<m_SimRobot.GetParts().size(); ii++){
    ModelNode* part = m_SimRobot.GetParts().at(ii);
    if (dynamic_cast<Shape*>(part)){
      Eigen::Vector6d newPose = m_PoseRW + part->GetPose();
      part->SetPose( newPose );
      m_Phys.RegisterObject( part );
    }
  }
}

void ModelGraphBuilder::AssociatePhysicsConstraints(SimRobot& m_SimRobot){
  for(unsigned int ii=0; ii<m_SimRobot.GetParts().size(); ii++){
    ModelNode* part = m_SimRobot.GetParts().at(ii);
    // Constraints are in their own reference frame, so they
    // don't need modification.
    if (dynamic_cast<Constraint*>(part)){
      m_Phys.RegisterObject( part );
    }
  }
}

void ModelGraphBuilder::AssociateRobotPhysics(SimRobot& m_SimRobot){
  AssociatePhysicsShapes(m_SimRobot);
  AssociatePhysicsConstraints(m_SimRobot);
}


void ModelGraphBuilder::AssociateDevices(SimDevices& m_SimDevices){
 for(map<string, SimDeviceInfo*>::iterator it =
      m_SimDevices.m_vSimDevices.begin();
      it != m_SimDevices.m_vSimDevices.end();
      it++){
    SimDeviceInfo* pDevice = it->second;
    m_Phys.RegisterDevice(pDevice);
  }
}

/////////////////////////////////////////

void ModelGraphBuilder::RenderWorldGraph(SimWorld m_SimWorld){
  for(unsigned int ii=0; ii<m_SimWorld.m_WorldNodes.size(); ii++){
    ModelNode* part = m_SimWorld.m_WorldNodes.at(ii);
    m_Render.AddNode(part);
  }
}

void ModelGraphBuilder::RenderRobotGraph(SimRobot m_SimRobot){
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

void ModelGraphBuilder::Init(SimWorld& m_WorldModel, SimRobot& m_SimRobot,
                             SimDevices& m_SimDevices,
                             std::string sSimName, bool debug, bool render){
  m_debug = debug;
  m_render = render;
  m_Phys.Init();
  if(m_render){
    m_Render.Init(sSimName);
  }
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
  AssociateDevices(m_SimDevices);
  if(m_render){
    RenderWorldGraph(m_WorldModel);
    RenderRobotGraph(m_SimRobot);
    m_Render.AddToScene();
    m_Render.CompleteScene();
    m_Render.AddDevices(m_SimDevices);
  }
}

void ModelGraphBuilder::UpdateScene(){
  m_Phys.StepSimulation();
  if(m_render){
    m_Render.UpdateScene();
  }
}




