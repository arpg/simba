#ifndef RENDERCLASS_H
#define RENDERCLASS_H

#include <map>

// Our SceneGraph interface
#include <SceneGraph/SceneGraph.h>
#include "ModelGraph/GLHeightmap.h"

// All of our bullet objects
#include <BulletStructs/Shape.h>
#include <BulletStructs/Constraint.h>
#include <BulletStructs/SimRaycastVehicle.h>

// Our Sensor data
#include <SimDevices/SimDevices.h>

class RenderEngine
{
public:

  void Init(std::string sLocalSimName);

  ///////////////////////

  // Add to our list of SceneEntities
  SceneGraph::GLObject* AddNode(ModelNode *pNode);

  ///////////////////////////////////////

  // Add sensors and cameras to the Scene
  void AddDevices(SimDevices& Devices);
  bool UpdateCameras();
  void SetImagesToWindow();

  ///////////////////////////////////////

  // Pass all SceneEntities (and RaycastWheels) at one time into
  // the Scene
  void AddToScene();
  void AddNewShape(SceneGraph::GLObject* object);

  // Complete the SceneGraph and Pangolin initialization
  void CompleteScene(bool bEnableCameraView);

  bool isCameraBody(string BodyName, string CameraName);

  // Update the Scene by one timestep
  void UpdateScene();


  /// MEMBER VARIABLES
  std::map<ModelNode*, SceneGraph::GLObject*> m_mSceneEntities;
  std::map<string, SceneGraph::GLObject*>     m_mRaycastWheels;
  std::map<SimCamera*, ModelNode*>            m_mCameras;
  SceneGraph::GLSceneGraph                    m_glGraph;
  SceneGraph::ImageView*                      m_LSimCamImage;
  SceneGraph::ImageView*                      m_RSimCamImage;
  pangolin::View*                             m_view3d;
  pangolin::OpenGlRenderState                 m_stacks3d;
  bool                                        m_bCameraView;

};

#endif // RENDERCLASS_H
