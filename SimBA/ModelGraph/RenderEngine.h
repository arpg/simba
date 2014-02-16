#ifndef RENDERCLASS_H
#define RENDERCLASS_H

#include <map>

// Our SceneGraph interface
#include <SceneGraph/SceneGraph.h>

// All of our bullet objects
#include <ModelGraph/Shape.h>
#include <ModelGraph/Constraint.h>
#include <ModelGraph/RaycastVehicle.h>

class RenderEngine
{
public:

  void Init(std::string sLocalSimName){
    //Start our SceneGraph interface
    pangolin::CreateGlutWindowAndBind(sLocalSimName,640,480);
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
    glClearColor(0, 0, 0, 1);
    glewInit();
  }


  //////////////////////////////
  //////////////////////////////

  // Add to our list of SceneEntities
  // This will be added later in 'AddToScene'
  void AddNode( ModelNode *pNode){

    // Add our RaycastVehicle (a myriad of shapes)
    if (dynamic_cast<RaycastVehicle*>(pNode) != NULL){

      // TODO: ADD RAYCAST VEHICLE

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
//        PlaneShape* pbShape = (PlaneShape *) pShape;
        SceneGraph::GLGrid* new_plane = new SceneGraph::GLGrid();
        new_plane->SetNumLines(20);
        new_plane->SetLineSpacing(1);
        m_mSceneEntities[pNode] = new_plane;
      }

      //Light
      else if (dynamic_cast<LightShape*>(pShape) != NULL){
        LightShape* pbShape = (LightShape *) pShape;
        SceneGraph::GLShadowLight* new_light = new SceneGraph::GLShadowLight();
        new_light->SetPose(pbShape->GetPose());
        new_light->EnableLight();
        m_mSceneEntities[pNode] = new_light;
      }

      //Mesh
      else if (dynamic_cast<MeshShape*>(pShape) != NULL){
        MeshShape* pbShape = (MeshShape *) pShape;
        SceneGraph::GLMesh* new_mesh =
            new SceneGraph::GLMesh(pbShape->GetFileDir());
        new_mesh->SetPose(pbShape->GetPose());
        m_mSceneEntities[pNode] = new_mesh;
      }

    }
  }

  ///////////////////////////////////////


  void AddToScene(){
    std::map<ModelNode*, SceneGraph::GLObject* >::iterator it;
    for(it = m_mSceneEntities.begin(); it != m_mSceneEntities.end(); it++) {
      ModelNode* node = it->first;
      cout<<"--)) "<<node->GetName()<<endl;
      SceneGraph::GLObject* p = it->second;
      m_glGraph.AddChild( p );
    }
  }

  void CompleteScene(){
    const SceneGraph::AxisAlignedBoundingBox bbox =
        m_glGraph.ObjectAndChildrenBounds();
    const Eigen::Vector3d center = bbox.Center();
    const double size = bbox.Size().norm();
    const double far = 2*size;
    const double near = far / 1E3;

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState stacks(
          pangolin::ProjectionMatrix(640,480,420,420,320,240,near,far),
          pangolin::ModelViewLookAt(center(0), center(1) + size,
                                    center(2) - size/4,
                                    center(0), center(1), center(2),
                                    pangolin::AxisNegZ) );
    m_stacks3d = stacks;

    // We define a new view which will reside within the container.

    // We set the views location on screen and add a handler which will
    // let user input update the model_view matrix (stacks3d) and feed through
    // to our scenegraph
    m_view3d = new SceneGraph::ImageView(false, true);
    m_view3d->SetBounds( 0.0, 1.0, 0.0, 0.75/*, -640.0f/480.0f*/ );
    m_view3d->SetHandler( new SceneGraph::HandlerSceneGraph(
                            m_glGraph, m_stacks3d) );
    m_view3d->SetDrawFunction( SceneGraph::ActivateDrawFunctor(
                                 m_glGraph, m_stacks3d) );

    // window for display image capture from SimCamera
    m_LSimCamImage = new SceneGraph::ImageView(true, true);
    m_LSimCamImage->SetBounds( 0.0, 0.5, 0.5, 1.0/*, 512.0f/384.0f*/ );

    // window for display image capture from SimCamera
    m_RSimCamImage = new SceneGraph::ImageView(true, true);
    m_RSimCamImage->SetBounds( 0.5, 1.0, 0.5, 1.0/*, 512.0f/384.0f */);


    // Add our views as children to the base container.
    pangolin::DisplayBase().AddDisplay( *m_view3d );
    pangolin::DisplayBase().AddDisplay( *m_LSimCamImage );
    pangolin::DisplayBase().AddDisplay( *m_RSimCamImage );

  }

  void UpdateScene(){
    std::map<ModelNode*, SceneGraph::GLObject*>::iterator it;
    for(it=m_mSceneEntities.begin(); it != m_mSceneEntities.end(); it++) {
      ModelNode* mn = it->first;
      cout<<"WOOOOOO"<<endl;
      cout<<mn->GetPose()<<endl;
      SceneGraph::GLObject* p = it->second;
      p->SetPose( mn->GetPose() );
    }
  }

  std::map<ModelNode*, SceneGraph::GLObject*> m_mSceneEntities;
  SceneGraph::GLSceneGraph                    m_glGraph;
  SceneGraph::ImageView*                      m_LSimCamImage;
  SceneGraph::ImageView*                      m_RSimCamImage;
  SceneGraph::ImageView*                      m_view3d;
  pangolin::OpenGlRenderState                 m_stacks3d;


};

#endif // RENDERCLASS_H
