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
/// DebugDraw class
/// This is for debugging the Bullet physics system. There are times when it's
/// not quite certain what your shapes are doing in the ModelGraph. This
/// outlines the object as Bullet sees it (not ModelGraph), so that the user can
/// debug its rendering.
///
//////////////////////////////////////////////////////////

class DebugDraw : public btIDebugDraw
{
  int m_debugMode;
public:

  virtual void drawLine(const btVector3& from, const btVector3& to,
                        const btVector3& fromColor, const btVector3& toColor){
    glBegin(GL_LINES);
    glColor3f(fromColor.getX(), fromColor.getY(), fromColor.getZ());
    glVertex3d(from.getX(), from.getY(), from.getZ());
    glColor3f(toColor.getX(), toColor.getY(), toColor.getZ());
    glVertex3d(to.getX(), to.getY(), to.getZ());
    glEnd();
  }

  ///////////////////

  virtual void drawLine(const btVector3& from, const btVector3& to,
                         const btVector3& color){
    drawLine(from,to,color,color);
  }

  ///////////////////

  virtual int getDebugMode( void ) const{
    return m_debugMode;
  }

  ///////////////////

  virtual void drawSphere (const btVector3& p, btScalar radius,
                           const btVector3& color){
    glColor4f (color.getX(), color.getY(), color.getZ(), btScalar(1.0f));
    glPushMatrix ();
    glTranslatef (p.getX(), p.getY(), p.getZ());

    int lats = 5;
    int longs = 5;

    int i, j;
    for(i = 0; i <= lats; i++) {
      btScalar lat0 = SIMD_PI * (-btScalar(0.5) + (btScalar) (i - 1) / lats);
      btScalar z0  = radius*sin(lat0);
      btScalar zr0 =  radius*cos(lat0);

      btScalar lat1 = SIMD_PI * (-btScalar(0.5) + (btScalar) i / lats);
      btScalar z1 = radius*sin(lat1);
      btScalar zr1 = radius*cos(lat1);

      glBegin(GL_QUAD_STRIP);
      for(j = 0; j <= longs; j++) {
        btScalar lng = 2 * SIMD_PI * (btScalar) (j - 1) / longs;
        btScalar x = cos(lng);
        btScalar y = sin(lng);

        glNormal3f(x * zr0, y * zr0, z0);
        glVertex3f(x * zr0, y * zr0, z0);
        glNormal3f(x * zr1, y * zr1, z1);
        glVertex3f(x * zr1, y * zr1, z1);
      }
      glEnd();
    }

    glPopMatrix();
  }

  ///////////////////

  virtual void drawBox (const btVector3& boxMin, const btVector3& boxMax,
                        const btVector3& color, btScalar alpha){
    btVector3 halfExtent = (boxMax - boxMin) * btScalar(0.5f);
    btVector3 center = (boxMax + boxMin) * btScalar(0.5f);
    //glEnable(GL_BLEND);     // Turn blending On
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glColor4f (color.getX(), color.getY(), color.getZ(), alpha);
    glPushMatrix ();
    glTranslatef (center.getX(), center.getY(), center.getZ());
    glScaled(2*halfExtent[0], 2*halfExtent[1], 2*halfExtent[2]);
    //	glutSolidCube(1.0);
    glPopMatrix ();
    //glDisable(GL_BLEND);
  }

  ///////////////////

  virtual void	drawTriangle(const btVector3& a, const btVector3& b,
                             const btVector3& c, const btVector3& color,
                             btScalar alpha){
    const btVector3	n=btCross(b-a,c-a).normalized();
    glBegin(GL_TRIANGLES);
    glColor4f(color.getX(), color.getY(), color.getZ(),alpha);
    glNormal3d(n.getX(),n.getY(),n.getZ());
    glVertex3d(a.getX(),a.getY(),a.getZ());
    glVertex3d(b.getX(),b.getY(),b.getZ());
    glVertex3d(c.getX(),c.getY(),c.getZ());
    glEnd();
  }

  ///////////////////

  virtual void setDebugMode(int debugMode){
    m_debugMode = debugMode;
  }

  ///////////////////

  virtual void draw3dText(const btVector3& location,const char* textString){
  }

  ///////////////////

  virtual void reportErrorWarning(const char* warningString){
    printf("%s\n",warningString);
  }

  ///////////////////

  virtual void drawContactPoint(const btVector3& pointOnB,
                                const btVector3& normalOnB,
                                btScalar distance, int lifeTime,
                                const btVector3& color){
    btVector3 to=pointOnB+normalOnB*1;//distance;
    const btVector3&from = pointOnB;
    glColor4f(color.getX(), color.getY(), color.getZ(),1.f);
    //glColor4f(0,0,0,1.f);
    glBegin(GL_LINES);
    glVertex3d(from.getX(), from.getY(), from.getZ());
    glVertex3d(to.getX(), to.getY(), to.getZ());
    glEnd();
  }

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
