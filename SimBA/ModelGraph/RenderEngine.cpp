#include <ModelGraph/RenderEngine.h>


void RenderEngine::Init(std::string sLocalSimName){
  //Start our SceneGraph interface
  pangolin::CreateGlutWindowAndBind(sLocalSimName,640,480);
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  glClearColor(0, 0, 0, 1);
  glewInit();
}

///////////////////////

// Add to our list of SceneEntities
// This will be added later in 'AddToScene'
void RenderEngine::AddNode( ModelNode *pNode){

  // Add our RaycastVehicle (a myriad of shapes)
  if (dynamic_cast<SimRaycastVehicle*>(pNode) != NULL){
    // A RaycastVehicle is made of a cube and four cylinders.
    // Since a vehicle is one shape in the PhysicsEngine, but five shapes
    // in the RenderEngine, we must call PhysicsEngine::GetVehicleTransform
    // anytime we want to update the rendering.

    // Get the positions of every part of the car.

    SimRaycastVehicle* pVehicle = (SimRaycastVehicle*) pNode;
    std::vector<double> params = pVehicle->GetParameters();

    // Were the meshes set for the car? If so, import those; else, use shapes.
    if(pVehicle->GetBodyMesh()!="NONE" && pVehicle->GetWheelMesh()!="NONE"){

      // The chassis
      SceneGraph::GLMesh* chassis_mesh = new SceneGraph::GLMesh();
      chassis_mesh->Init(pVehicle->GetBodyMesh());
      chassis_mesh->SetPerceptable(true);
      Eigen::Vector3d vScale;
      Eigen::Vector3d vBodyDim = pVehicle->GetBodyMeshDim();
      vScale<<(params[WheelBase]/vBodyDim(1)),
          1.5*params[Width]/vBodyDim(0),
          params[Height]/vBodyDim(2);
      chassis_mesh->SetScale(vScale);
      chassis_mesh->SetPose(pVehicle->GetPose());
      m_mSceneEntities[pNode] = chassis_mesh;

      Eigen::Vector3d vWheelScale;
      Eigen::Vector3d vWheelDim = pVehicle->GetWheelMeshDim();
      vWheelScale<<2*params[WheelRadius]/vWheelDim(1),
          2*params[WheelRadius]/vWheelDim(2),
          params[WheelWidth]/vWheelDim(0);
      cout<<vWheelScale<<endl;

      // FL Wheel
      SceneGraph::GLMesh* FLWheel = new SceneGraph::GLMesh();
      FLWheel->Init(pVehicle->GetWheelMesh());
      FLWheel->SetScale(vWheelScale);
      FLWheel->SetPose(pVehicle->GetWheelPose(0));
      m_mRaycastWheels[pVehicle->GetName()+"@FLWheel"] = FLWheel;

      // FR Wheel
      SceneGraph::GLMesh* FRWheel = new SceneGraph::GLMesh();
      FRWheel->Init(pVehicle->GetWheelMesh());
      FRWheel->SetPose(pVehicle->GetWheelPose(1));
      FRWheel->SetScale(vWheelScale);
      m_mRaycastWheels[pVehicle->GetName()+"@FRWheel"] = FRWheel;

      // BL Wheel
      SceneGraph::GLMesh* BLWheel = new SceneGraph::GLMesh();
      BLWheel->Init(pVehicle->GetWheelMesh());
      BLWheel->SetPose(pVehicle->GetWheelPose(2));
      BLWheel->SetScale(vWheelScale);
      m_mRaycastWheels[pVehicle->GetName()+"@BLWheel"] = BLWheel;

      // BR Wheel
      SceneGraph::GLMesh* BRWheel = new SceneGraph::GLMesh();
      BRWheel->Init(pVehicle->GetWheelMesh());
      BRWheel->SetPose(pVehicle->GetWheelPose(3));
      BRWheel->SetScale(vWheelScale);
      m_mRaycastWheels[pVehicle->GetName()+"@BRWheel"] = BRWheel;
    }

    else{
      // The chassis
      SceneGraph::GLBox* chassis = new SceneGraph::GLBox();
      chassis->SetExtent(params[WheelBase], params[Width], params[Height]);
      chassis->SetPose(pVehicle->GetPose());
      m_mSceneEntities[pNode] = chassis;

      // FL Wheel
      SceneGraph::GLCylinder* FLWheel = new SceneGraph::GLCylinder();
      FLWheel->Init(params[WheelRadius], params[WheelRadius],
                    params[WheelWidth], 10, 10);
      FLWheel->SetPose(pVehicle->GetWheelPose(0));
      m_mRaycastWheels[pVehicle->GetName()+"@FLWheel"] = FLWheel;

      // FR Wheel
      SceneGraph::GLCylinder* FRWheel = new SceneGraph::GLCylinder();
      FRWheel->Init(params[WheelRadius], params[WheelRadius],
                    params[WheelWidth], 10, 10);
      FRWheel->SetPose(pVehicle->GetWheelPose(1));
      m_mRaycastWheels[pVehicle->GetName()+"@FRWheel"] = FRWheel;

      // BL Wheel
      SceneGraph::GLCylinder* BLWheel = new SceneGraph::GLCylinder();
      BLWheel->Init(params[WheelRadius], params[WheelRadius],
                    params[WheelWidth], 10, 10);
      BLWheel->SetPose(pVehicle->GetWheelPose(2));
      m_mRaycastWheels[pVehicle->GetName()+"@BLWheel"] = BLWheel;

      // BR Wheel
      SceneGraph::GLCylinder* BRWheel = new SceneGraph::GLCylinder();
      BRWheel->Init(params[WheelRadius], params[WheelRadius],
                    params[WheelWidth], 10, 10);
      BRWheel->SetPose(pVehicle->GetWheelPose(3));
      m_mRaycastWheels[pVehicle->GetName()+"@BRWheel"] = BRWheel;
    }
  }

  // Add our Shapes
  if (dynamic_cast<Shape*>(pNode) != NULL){
    Shape* pShape = (Shape*)(pNode);

    //Box
    if (dynamic_cast<BoxShape*>(pShape) != NULL){
      BoxShape* pbShape = (BoxShape *) pShape;
      SceneGraph::GLBox* new_box = new SceneGraph::GLBox();
      new_box->SetExtent(pbShape->m_dBounds[0], pbShape->m_dBounds[1],
                         pbShape->m_dBounds[2]);
      new_box->SetPose(pbShape->GetPose());
      m_mSceneEntities[pNode] = new_box;
    }

    //Cylinder
    else if (dynamic_cast<CylinderShape*>(pShape) != NULL){
      CylinderShape* pbShape = (CylinderShape *) pShape;
      SceneGraph::GLCylinder* new_cylinder = new SceneGraph::GLCylinder();
      new_cylinder->Init(pbShape->m_dRadius, pbShape->m_dRadius,
                         pbShape->m_dHeight, 32, 1);
      new_cylinder->SetPose(pbShape->GetPose());
      m_mSceneEntities[pNode] = new_cylinder;
    }

    //Plane
    else if (dynamic_cast<PlaneShape*>(pShape) != NULL){
      PlaneShape* pbShape = (PlaneShape *) pShape;
      SceneGraph::GLGrid* new_plane = new SceneGraph::GLGrid();
      new_plane->SetNumLines(20);
      new_plane->SetLineSpacing(1);
      Eigen::Vector3d eig_norm;
      eig_norm<<pbShape->m_dNormal[0],
          pbShape->m_dNormal[1],
          pbShape->m_dNormal[2];
      new_plane->SetPlane(eig_norm);
      m_mSceneEntities[pNode] = new_plane;
    }

    //Light
    else if (dynamic_cast<LightShape*>(pShape) != NULL){
      LightShape* pbShape = (LightShape *) pShape;
//      SceneGraph::GLShadowLight* new_light = new SceneGraph::GLShadowLight();
      // IMPORTANT!! Can only use GLLight here if you want to use sim cam...
      // DO NOT USE GL Shadow HERE!!!
      SceneGraph::GLLight* new_light =
          new SceneGraph::GLLight(pbShape->GetPose()(0,0),
                                  pbShape->GetPose()(1,0),
                                  pbShape->GetPose()(2,0));
      m_mSceneEntities[pNode] = new_light;
    }

    //Mesh
    else if (dynamic_cast<MeshShape*>(pShape) != NULL){
      MeshShape* pbShape = (MeshShape *) pShape;
      SceneGraph::GLMesh* new_mesh = new SceneGraph::GLMesh();
      new_mesh->Init(pbShape->GetFileDir());
      new_mesh->SetPerceptable(true);
      new_mesh->SetScale(pbShape->GetScale());
      new_mesh->SetPose(pbShape->GetPose());
      m_mSceneEntities[pNode] = new_mesh;
    }

    //Heightmap
    else if (dynamic_cast<HeightmapShape*>(pShape) != NULL){
      HeightmapShape* pbShape = (HeightmapShape *) pShape;
      SceneGraph::GLHeightmap* new_map =
          new SceneGraph::GLHeightmap(pbShape->x_data_, pbShape->y_data_,
                                      pbShape->z_data_, pbShape->row_count_,
                                      pbShape->col_count_);
      m_mSceneEntities[pNode] = new_map;
    }
  }
}

///////////////////////////////////////

void RenderEngine::AddDevices(SimDevices& Devices){
  for(map<string, SimDeviceInfo*>::iterator it =
      Devices.m_vSimDevices.begin();
      it != Devices.m_vSimDevices.end();
      it++){
    SimDeviceInfo* Device = it->second;
    cout<<"[RenderEngine::AddDevices] "<<Device->m_sDeviceType<<endl;
    if(Device->m_sDeviceType=="Camera"){
      SimCamera* pSimCam = (SimCamera*) Device;
      // Initialize the cameras with SceneGraph
      pSimCam->init(&m_glGraph);
      // Match devices with their ModelNodes
      for(map<ModelNode*, SceneGraph::GLObject*>::iterator jj =
          m_mSceneEntities.begin();
          jj != m_mSceneEntities.end();
          jj++){
        ModelNode* pNode = jj->first;
        if(isCameraBody(pNode->GetName(), pSimCam->GetBodyName())){
          m_mCameras[pSimCam] = pNode;
        }
      }
    }
  }
}

/////////////////////////////////////////////////

bool RenderEngine::UpdateCameras()
{
  bool bStatus = false;
  for(map<SimCamera*, ModelNode*>::iterator it = m_mCameras.begin();
      it != m_mCameras.end();
      it++){
    SimCamera* Device = it->first;
    Device->Update();
    bStatus = true;
  }
    return bStatus;
}


/////////////////////////////////////////////////

// Scan all SimDevices and send the simulated camera images to Pangolin.
// Right now, we can only support up to two windows.

// TODO: Fairly certain that there's a glitch here. Go through and correct the
// Image buffers for content.
void RenderEngine::SetImagesToWindow(){
  int WndCounter = 0;
  for(map<SimCamera*, ModelNode*>::iterator it = m_mCameras.begin();
      it != m_mCameras.end();
      it++){
    SimCamera* Device = it->first;
    if(Device->m_bDeviceOn==true){
      SimCamera* pSimCam = (SimCamera*) Device;
      SceneGraph::ImageView* ImageWnd;
      // get pointer to window
      // TODO: Accept more windows. We might have more cameras, who knows.
      (WndCounter == 0) ? ImageWnd = m_LSimCamImage :
          ImageWnd = m_RSimCamImage;
      WndCounter++;
      // Set image to window
      // DEPTH
      if(pSimCam->m_iCamType == SceneGraph::eSimCamDepth){
        float* pImgbuf = (float*) malloc( pSimCam->m_nImgWidth *
                                          pSimCam->m_nImgHeight *
                                          sizeof(float) );
        bool success = pSimCam->capture(pImgbuf);
        if(success){
          ImageWnd->SetImage(pImgbuf, pSimCam->m_nImgWidth,
                             pSimCam->m_nImgHeight,
                             GL_INTENSITY, GL_LUMINANCE, GL_FLOAT);
          free(pImgbuf);
        }
      }
      // RGB
      else if(pSimCam->m_iCamType == SceneGraph::eSimCamRGB){
        char* pImgbuf= (char*)malloc (pSimCam->m_nImgWidth *
                                      pSimCam->m_nImgHeight * 3);
        if(pSimCam->capture(pImgbuf)==true){
          ImageWnd->SetImage(pImgbuf, pSimCam->m_nImgWidth,
                             pSimCam->m_nImgHeight,
                             GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE);
          free(pImgbuf);
        }
      }
      // GREY
      else if(pSimCam->m_iCamType == SceneGraph::eSimCamLuminance){
        char* pImgbuf= (char*)malloc (pSimCam->m_nImgWidth *
                                      pSimCam->m_nImgHeight);
        if(pSimCam->capture(pImgbuf)==true){
          ImageWnd->SetImage(pImgbuf, pSimCam->m_nImgWidth,
                             pSimCam->m_nImgHeight,
                             GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE);
          free(pImgbuf);
        }
      }
    }
  }
}

///////////////////////////////////////

void RenderEngine::AddToScene(){

  // Add shapes
  std::map<ModelNode*, SceneGraph::GLObject* >::iterator it;
  for(it = m_mSceneEntities.begin(); it != m_mSceneEntities.end(); it++) {
    SceneGraph::GLObject* p = it->second;
    m_glGraph.AddChild( p );
  }
  // If we have them, add wheels
  std::map<string, SceneGraph::GLObject* >::iterator jj;
  for(jj = m_mRaycastWheels.begin(); jj != m_mRaycastWheels.end(); jj++) {
    SceneGraph::GLObject* w = jj->second;
    m_glGraph.AddChild( w );
  }
}

///////////////////////////////////////

void RenderEngine::CompleteScene(bool bEnableCameraView=false)
{
  const SceneGraph::AxisAlignedBoundingBox bbox =
      m_glGraph.ObjectAndChildrenBounds();
  const Eigen::Vector3d center = bbox.Center();
  const double size = bbox.Size().norm();
  const double far = 15*size;
  const double near = far / 1E3;

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState stacks(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,near,far),
        pangolin::ModelViewLookAt(center(0), center(1) + 7,
                                  center(2) + 6,
                                  center(0), center(1), center(2),
                                  pangolin::AxisZ) );
  m_stacks3d = stacks;

  // We define a new view which will reside within the container.

  // We set the views location on screen and add a handler which will
  // let user input update the model_view matrix (stacks3d) and feed through
  // to our scenegraph
  m_view3d = new pangolin::View(0.0);
  m_view3d->SetBounds( 0.0, 1.0, 0.0, 1.0/*, -640.0f/480.0f*/ );
  m_view3d->SetHandler( new SceneGraph::HandlerSceneGraph(
                          m_glGraph, m_stacks3d) );
  m_view3d->SetDrawFunction( SceneGraph::ActivateDrawFunctor(
                               m_glGraph, m_stacks3d) );

  // Add our views as children to the base container.
  pangolin::DisplayBase().AddDisplay( *m_view3d );

  if(bEnableCameraView==true){
      m_bCameraView= true;
    //  // window for display image capture from SimCamera
      m_LSimCamImage = new SceneGraph::ImageView(true, true);
      m_LSimCamImage->SetBounds( 0.0, 0.5, 0.75, 1.0/*, 512.0f/384.0f*/ );

    //  // window for display image capture from SimCamera
      m_RSimCamImage = new SceneGraph::ImageView(true, true);
      m_RSimCamImage->SetBounds( 0.5, 1.0, 0.75, 1.0/*, 512.0f/384.0f */);

      pangolin::DisplayBase().AddDisplay( *m_LSimCamImage );
      pangolin::DisplayBase().AddDisplay( *m_RSimCamImage );
  }
  else
  {
    m_bCameraView= false;
  }
}

////////////////////////////////////////////////////

bool RenderEngine::isCameraBody(string BodyName, string CameraName){
  bool inthere = false;
  std::size_t found = BodyName.find(CameraName);
  if (found!=std::string::npos){
    inthere = true;
  }
  return inthere;
}

////////////////////////////////////////////////////

void RenderEngine::UpdateScene(){
  /// React to changes in the PhysicsEngine
  std::map<ModelNode*, SceneGraph::GLObject*>::iterator it;
  for(it=m_mSceneEntities.begin(); it != m_mSceneEntities.end(); it++) {
    ModelNode* mn = it->first;
    SceneGraph::GLObject* p = it->second;
    p->SetPose( mn->GetPose() );
    // Update all of our tires.
    if((dynamic_cast<SimRaycastVehicle*>(mn) != NULL)){
      std::map<string, SceneGraph::GLObject*>::iterator jj;
      SimRaycastVehicle* pVehicle = (SimRaycastVehicle*) mn;
      for(jj=m_mRaycastWheels.begin(); jj != m_mRaycastWheels.end(); jj++) {
        string name = jj->first;
        SceneGraph::GLObject* wheel = jj->second;
        if(name == pVehicle->GetName()+"@FLWheel"){
          wheel->SetPose(pVehicle->GetWheelPose(0));
        }
        else if(name == pVehicle->GetName()+"@FRWheel"){
          wheel->SetPose(pVehicle->GetWheelPose(1));
        }
        else if(name == pVehicle->GetName()+"@BLWheel"){
          wheel->SetPose(pVehicle->GetWheelPose(2));
        }
        else if(name == pVehicle->GetName()+"@BRWheel"){
          wheel->SetPose(pVehicle->GetWheelPose(3));
        }
      }
    }
  }
  /// Change the views in the Cameras
  std::map<SimCamera*, ModelNode*>::iterator jj;
  for(jj=m_mCameras.begin(); jj != m_mCameras.end(); jj++) {
    SimCamera* pCamera = jj->first;
    ModelNode* pNode = jj->second;
    pCamera->m_vPose = pNode->GetPose();
  }
  if(UpdateCameras()==true &&m_bCameraView==true){
    SetImagesToWindow();
  }
}
