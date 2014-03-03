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


/////////////////////////////////////////////////

void ModelGraphBuilder::ConstructDevices(SimDevices m_SimDevices){


}


/////////////////////////////////////////////////

// Scan all SimDevices and send the simulated camera images to Pangolin.
// Right now, we can only support up to two windows.

//TODO: Fairly certain that there's a glitch here. Go through and correct the
//Image buffers for content.
bool ModelGraphBuilder::SetImagesToWindow(SceneGraph::ImageView& LSimCamWnd,
                                          SceneGraph::ImageView& RSimCamWnd ){
  int WndCounter = 0;

  for(unsigned int i =0 ; i!= m_SimDevices.m_vSimDevices.size(); i++){
    SimDeviceInfo Device = m_SimDevices.m_vSimDevices[i];
    if(Device.m_bDeviceOn==true){
      for(unsigned int j=0;j!=Device.m_vSensorList.size();j++){
        string sSimCamName = Device.m_vSensorList[j];
        SimCamera* pSimCam = m_SimDevices.GetSimCam(sSimCamName);
        SceneGraph::ImageView* ImageWnd;
        // get pointer to window
        if(WndCounter == 0){
          ImageWnd = &LSimCamWnd;
        }
        else if(WndCounter == 1){
          ImageWnd = &RSimCamWnd;
        }

        WndCounter++;
        // set image to window
        if (pSimCam->m_iCamType == 5){       // for depth image
          float* pImgbuf = (float*) malloc( pSimCam->m_nImgWidth *
                                            pSimCam->m_nImgHeight *
                                            sizeof(float) );
          if(pSimCam->capture(pImgbuf)==true){
            ImageWnd->SetImage(pImgbuf, pSimCam->m_nImgWidth,
                               pSimCam->m_nImgHeight,
                               GL_INTENSITY, GL_LUMINANCE, GL_FLOAT);
            free(pImgbuf);
          }
          else{
            cout<<"[SetImagesToWindow] Set depth Image fail"<<endl;
            return false;
          }
        }
        else if(pSimCam->m_iCamType == 2){   // for RGB image
          char* pImgbuf= (char*)malloc (pSimCam->m_nImgWidth *
                                        pSimCam->m_nImgHeight * 3);
          if(pSimCam->capture(pImgbuf)==true){
            ImageWnd->SetImage(pImgbuf, pSimCam->m_nImgWidth,
                               pSimCam->m_nImgHeight,
                               GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE);
            free(pImgbuf);
          }
          else{
            cout<<"[SetImagesToWindow] Set RGB Image fail"<<endl;
            return false;
          }
        }
        else if(pSimCam->m_iCamType == 1){    //to show greyscale image
          char* pImgbuf= (char*)malloc (pSimCam->m_nImgWidth *
                                        pSimCam->m_nImgHeight);
          if(pSimCam->capture(pImgbuf)==true){
            ImageWnd->SetImage(pImgbuf, pSimCam->m_nImgWidth,
                               pSimCam->m_nImgHeight,
                               GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE);
            free(pImgbuf);
          }
          else{
            cout<<"[SetImagesToWindow] Set Gray Image fail"<<endl;
            return false;
          }
        }
      }
    }
  }
  return true;
}


////////////////////////////////////////

void ModelGraphBuilder::Init(SimWorld& m_WorldModel, SimRobot& m_SimRobot,
                             SimDevices& m_SimDevices,
                             std::string sSimName, bool debug){
  m_debug = debug;
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

void ModelGraphBuilder::UpdateScene(){
  m_Phys.StepSimulation();
  m_Render.UpdateScene();
}




