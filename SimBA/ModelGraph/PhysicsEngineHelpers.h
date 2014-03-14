#ifndef PHYSICSENGINEHELPERS_H
#define PHYSICSENGINEHELPERS_H

#include <math.h>
#include <boost/shared_ptr.hpp>

// Bullet libraries
#include <bullet/LinearMath/btIDebugDraw.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include <bullet/btBulletDynamicsCommon.h>

// Our ModelNode Objects
#include <ModelGraph/Shape.h>
#include <ModelGraph/RaycastVehicle.h>
#include <ModelGraph/Constraint.h>
#include <SimDevices/SimDeviceInfo.h>

//SceneGraphMotionState (for tracking our shapes)
#include <SceneGraph/SceneGraph.h>
#include <pangolin/pangolin.h>
#include <ModelGraph/GLDebugDrawer.h>

//Assimp to import our meshes into the Physics system
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

enum Compounds{
  VEHICLE = 0
};

//////////////////////////////////////////////////////////
///
/// NodeMotionState class
///
//////////////////////////////////////////////////////////

class NodeMotionState : public btMotionState {
public:
  NodeMotionState(ModelNode& obj)
    : object(obj){
  }

  virtual void getWorldTransform(btTransform &worldTrans) const {
    worldTrans = toBullet(object.GetPoseMatrix());
  }

  virtual void setWorldTransform(const btTransform &worldTrans) {
    object.SetPose(toEigen(worldTrans));
  }

  ModelNode& object;

};

//////////////////////////////////////////////////////////
///
/// We declare these typedefs to shorten the name, basically.
///
//////////////////////////////////////////////////////////

typedef  boost::shared_ptr<btCollisionShape>            CollisionShapePtr;
typedef  boost::shared_ptr<btRigidBody>                 RigidBodyPtr;
typedef  boost::shared_ptr<NodeMotionState>             MotionStatePtr;
typedef  boost::shared_ptr<btRaycastVehicle>            VehiclePtr;

///////////////////////////////////////////////////////
///
/// The Entity class
/// Holds our bullet shapes and terrain.
///
///////////////////////////////////////////////////////

class Entity
{
public:
  Entity(){
  }

  Entity(CollisionShapePtr  pShape, MotionStatePtr  pMotionState,
         RigidBodyPtr pRigidShape, string sName){
    m_sName        = sName;
    m_pShape       = pShape;
    m_pMotionState = pMotionState;
    m_pRigidBody   = pRigidShape;
  }

  //member variables
  string                  m_sName;
  CollisionShapePtr       m_pShape;
  MotionStatePtr          m_pMotionState;
  RigidBodyPtr            m_pRigidBody;
};

///////////////////////////////////////////////////////
///
/// The Compound_Entity class
/// Holds all of our compound shapes and constraints, as well as the type of
/// compound we have.
///
///////////////////////////////////////////////////////

class Compound_Entity
{
public:
  Compound_Entity()
  {

  }
  Compound_Entity(double* Shape_ids,
                  double* Con_ids,
                  Compounds type,
                  string sName
                  ){
    m_sName        = sName;
    m_vShape_ids = Shape_ids;
    m_vCon_ids = Con_ids;
    m_Type = type;
  }
  string    m_sName;
  double*   m_vShape_ids;
  double*   m_vCon_ids;
  Compounds m_Type;
};

///////////////////////////////////////////////////////
///
/// The Vehicle_Entity class
/// Holds all of our RaycastVehicles
///
///////////////////////////////////////////////////////

class Vehicle_Entity
{
public:
  Vehicle_Entity()
  {

  }
  Vehicle_Entity(CollisionShapePtr  pShape, MotionStatePtr  pMotionState,
                 RigidBodyPtr pRigidShape, VehiclePtr pVehicle, string sName){
    m_sName        = sName;
    m_pShape       = pShape;
    m_pMotionState = pMotionState;
    m_pRigidBody   = pRigidShape;
    m_pVehicle     = pVehicle;
  }
  //member variables
  string                  m_sName;
  CollisionShapePtr       m_pShape;
  MotionStatePtr          m_pMotionState;
  RigidBodyPtr            m_pRigidBody;
  VehiclePtr              m_pVehicle;
};

#endif // PHYSICSENGINEHELPERS_H
