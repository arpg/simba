#include <ModelGraph/ModelGraphBuilder.h>

/////////////////////////////////////////
/// PHYSICS FUNCTIONS
/////////////////////////////////////////

void ModelGraphBuilder::AssociateWorldPhysics(SimWorld m_SimWorld) {
  for (unsigned int ii = 0; ii < m_SimWorld.m_WorldNodes.size(); ii++) {
    ModelNode* part = m_SimWorld.m_WorldNodes.at(ii);
    if (dynamic_cast<Shape*>(part)) {
      physics_engine_.RegisterObject( part );
    }
  }
}

void ModelGraphBuilder::AssociatePhysicsShapes(SimRobot& m_SimRobot) {
  for (unsigned int ii = 0; ii < m_SimRobot.GetParts().size(); ii++) {
    ModelNode* part = m_SimRobot.GetParts().at(ii);
    if (dynamic_cast<Shape*>(part)) {
      Eigen::Vector6d newPose = robot_pose_ + part->GetPose();
      part->SetPose( newPose );
      physics_engine_.RegisterObject( part );
    }
  }
}

void ModelGraphBuilder::AssociatePhysicsConstraints(SimRobot& m_SimRobot) {
  for (unsigned int ii = 0; ii < m_SimRobot.GetParts().size(); ii++) {
    ModelNode* part = m_SimRobot.GetParts().at(ii);
    // Constraints are in their own reference frame, so they
    // don't need modification.
    if (dynamic_cast<Constraint*>(part)) {
      physics_engine_.RegisterObject( part );
    }
  }
}

void ModelGraphBuilder::AssociateRobotPhysics(SimRobot& m_SimRobot) {
  AssociatePhysicsShapes(m_SimRobot);
  AssociatePhysicsConstraints(m_SimRobot);
}


void ModelGraphBuilder::AssociateDevices(SimDevices& m_SimDevices) {
 for (map<string, SimDeviceInfo*>::iterator it =
      m_SimDevices.m_vSimDevices.begin();
      it != m_SimDevices.m_vSimDevices.end();
      it++) {
    SimDeviceInfo* pDevice = it->second;
    physics_engine_.RegisterDevice(pDevice);
  }
}

/////////////////////////////////////////
/// RENDER FUNCTIONS
/////////////////////////////////////////

void ModelGraphBuilder::RenderWorldGraph(SimWorld m_SimWorld) {
  for (unsigned int ii = 0; ii < m_SimWorld.m_WorldNodes.size(); ii++) {
    ModelNode* part = m_SimWorld.m_WorldNodes.at(ii);
    render_engine_.AddNode(part);
  }
}

void ModelGraphBuilder::RenderRobotGraph(SimRobot m_SimRobot) {
  for (unsigned int ii = 0; ii < m_SimRobot.GetParts().size(); ii++) {
    ModelNode* part = m_SimRobot.GetParts().at(ii);
    render_engine_.AddNode(part);
    Eigen::Vector6d ChildWorldPose;
    for (unsigned int ii = 0; ii < part->NumChildren(); ii++ ) {
      ChildWorldPose = _T2Cart(
          part->GetPoseMatrix() * part->m_vChildren[ii]->GetPoseMatrix());
      part->SetPose(ChildWorldPose);
    }
  }
}

/////////////////////////////////////////
/// UPDATE FUNCTIONS
/// This just adds to the RenderGraph right now.
/// I'll worry about the Physics later
/////////////////////////////////////////

void ModelGraphBuilder::CheckForNewShapes() {
  std::mutex modelgraph_mutex;
  std::lock_guard<std::mutex> lock(modelgraph_mutex);
  int new_parts = sim_robot_->GetNewPartsBit();
  if (new_parts > 0) {
    for (unsigned int ii = new_parts;
         ii < sim_robot_->GetParts().size(); ii++) {
      ModelNode* part = sim_robot_->GetParts().at(ii);
      SceneGraph::GLObject* object = render_engine_.AddNode(part);
      render_engine_.AddNewShape(object);
      Eigen::Vector6d ChildWorldPose;
      for (unsigned int ii = 0; ii < part->NumChildren(); ii++ ) {
        ChildWorldPose = _T2Cart(
            part->GetPoseMatrix() * part->m_vChildren[ii]->GetPoseMatrix());
        part->SetPose(ChildWorldPose);
      }
    }
    sim_robot_->SetNewPartsBit(0);
  }
}

////////////////////////////////////////

void ModelGraphBuilder::Init(SimWorld& m_WorldModel, SimRobot& m_SimRobot,
                             SimDevices& m_SimDevices,
                             std::string sSimName, bool debug, bool render,
                             bool bEnableCameraView=false) {
  debug_status_ = debug;
  render_status_ = render;
  world_model_ = &m_WorldModel;
  sim_robot_ = &m_SimRobot;
  physics_engine_.Init();
  if (render_status_) {
    render_engine_.Init(sSimName);
  }
  if (m_SimRobot.GetStateKeeperStatus()) {
    // Get the PoseRW from the StateKeeper, and not the World.XML file.
  } else {
    robot_pose_ << m_WorldModel.m_vRobotPose[0], m_WorldModel.m_vRobotPose[1],
        m_WorldModel.m_vRobotPose[2], m_WorldModel.m_vRobotPose[3],
        m_WorldModel.m_vRobotPose[4], m_WorldModel.m_vRobotPose[5];
  }
  // We move the parts around to their respective places in
  // Associate****Physics; RenderWorldGraph should keep the same places.
  // This inheritance pattern is what makes the ModelNode class so important.
  AssociateWorldPhysics(m_WorldModel);
  AssociateRobotPhysics(m_SimRobot);
  AssociateDevices(m_SimDevices);
  if (render_status_) {
    RenderWorldGraph(m_WorldModel);
    RenderRobotGraph(m_SimRobot);
    render_engine_.AddToScene();
    render_engine_.CompleteScene(bEnableCameraView);
    render_engine_.AddDevices(m_SimDevices);
  }
}

//////////

void ModelGraphBuilder::UpdateScene() {
  CheckForNewShapes();
  physics_engine_.StepSimulation();
  if (debug_status_) {
    physics_engine_.DebugDrawWorld();
  } else {
    physics_engine_.StepSimulation();
  }
  if (render_status_) {
    render_engine_.UpdateScene();
  }
}
