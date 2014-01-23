#ifndef RENDERCLASS_H
#define RENDERCLASS_H

#include <map>

// Our SceneGraph interface
#include <SceneGraph/SceneGraph.h>

// All of our bullet objects
#include <ModelGraph/Shape.h>
#include <ModelGraph/Constraint.h>
#include <ModelGraph/RaycastVehicle.h>

class Render
{
public:

  Render(){
  }

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
    }
  }

  void AddToScene( SceneGraph::GLSceneGraph *glGraph ){
    std::map<ModelNode*, SceneGraph::GLObject* >::iterator it;
    for(it = m_mSceneEntities.begin(); it != m_mSceneEntities.end(); it++) {
      SceneGraph::GLObject* p = it->second;
      glGraph->AddChild( p );
    }
  }

  void UpdateScene( void )
  {
    std::map<ModelNode*, SceneGraph::GLObject*>::iterator it;
    for(it=m_mSceneEntities.begin(); it != m_mSceneEntities.end(); it++) {
      ModelNode* mn = it->first;
      SceneGraph::GLObject* p = it->second;
      p->SetPose( mn->GetPose() );
    }
  }

  //std::vector<SceneGraph::GLObject*> objects;
  std::map<ModelNode*, SceneGraph::GLObject*> m_mSceneEntities;

};

#endif // RENDERCLASS_H
