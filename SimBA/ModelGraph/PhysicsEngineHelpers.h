#ifndef PHYSICSENGINEHELPERS_H
#define PHYSICSENGINEHELPERS_H

#include <boost/shared_ptr.hpp>
#include <bullet/btBulletDynamicsCommon.h>
#include <ModelGraph/Models.h>
#include <ModelGraph/SE3.h>
#include <bullet/LinearMath/btIDebugDraw.h>
#include <math.h>

using namespace std;

//////////////////////////////////////////////////////////
///
/// EIGEN-TO-BULLET-TO-EIGEN CONVERTERS
///
//////////////////////////////////////////////////////////

inline Eigen::Matrix4d
getInverseTransformation (const Eigen::Matrix4d &transformation)
{
  Eigen::Matrix4d transformation_inverse;
  float tx = transformation (0, 3);
  float ty = transformation (1, 3);
  float tz = transformation (2, 3);

  transformation_inverse (0, 0) = transformation (0, 0);
  transformation_inverse (0, 1) = transformation (1, 0);
  transformation_inverse (0, 2) = transformation (2, 0);
  transformation_inverse (0, 3) = - (transformation (0, 0) * tx + transformation (0, 1) * ty + transformation (0, 2) * tz);


  transformation_inverse (1, 0) = transformation (0, 1);
  transformation_inverse (1, 1) = transformation (1, 1);
  transformation_inverse (1, 2) = transformation (2, 1);
  transformation_inverse (1, 3) = - (transformation (1, 0) * tx + transformation (1, 1) * ty + transformation (1, 2) * tz);

  transformation_inverse (2, 0) = transformation (0, 2);
  transformation_inverse (2, 1) = transformation (1, 2);
  transformation_inverse (2, 2) = transformation (2, 2);
  transformation_inverse (2, 3) = - (transformation (2, 0) * tx + transformation (2, 1) * ty + transformation (2, 2) * tz);

  transformation_inverse (3, 0) = 0;
  transformation_inverse (3, 1) = 0;
  transformation_inverse (3, 2) = 0;
  transformation_inverse (3, 3) = 1;
  return transformation_inverse;
}

inline Eigen::Matrix<double,4,4> toEigen(const btTransform& T)
{
  Eigen::Matrix<btScalar,4,4> eT;
  T.getOpenGLMatrix(eT.data());
  return eT.cast<double>();
}

inline btTransform toBullet(const Eigen::Matrix<double,4,4>& T)
{
  btTransform bT;
  Eigen::Matrix<btScalar,4,4> eT = T.cast<btScalar>();
  bT.setFromOpenGLMatrix(eT.data());
  return bT;
}

inline btVector3 toBulletVec3(const Eigen::Vector3d& v)
{
  btVector3 bv;
  bv.setX(v(0));
  bv.setY(v(1));
  bv.setZ(v(2));
  return bv;
}

inline btVector3 toBulletVec3(const double x, const double y, const double z)
{
  btVector3 bv;
  bv.setX(x);
  bv.setY(y);
  bv.setZ(z);
  return bv;
}

//////////////////////////////////////////////////////////
///
/// NodeMotionState class
///
//////////////////////////////////////////////////////////


class NodeMotionState : public btMotionState {
public:
  NodeMotionState(ModelNode& obj, Eigen::Vector6d& wp)
    : object(obj)
  {
    m_WorldPose = _Cart2T(wp);
  }

  NodeMotionState(ModelNode& obj, Eigen::Matrix4d& wp)
    : object(obj), m_WorldPose(wp)
  {
  }

  virtual void getWorldTransform(btTransform &worldTrans) const {
    worldTrans = toBullet( m_WorldPose);
  }

  virtual void setWorldTransform(const btTransform &worldTrans) {
    if (dynamic_cast<Body*>(&object))
    {
      Body* pBody = (Body*) &object;
      if (dynamic_cast<CylinderShape*>(pBody->m_CollisionShape))
      {
        Eigen::Vector6d temp;
        temp << 0, 0, 0, M_PI / 2, 0, 0;
        Eigen::Matrix4d rot;
        rot = toEigen(worldTrans);
        rot = rot*_Cart2T(temp);
        m_WorldPose = rot;
        object.SetWPose(m_WorldPose);
      }
      else
      {
        m_WorldPose = toEigen(worldTrans);
        object.SetWPose(m_WorldPose);
      }
    }
    else
    {
      m_WorldPose = toEigen(worldTrans);
      object.SetWPose(m_WorldPose);
    }
  }


  ModelNode& object;
  Eigen::Matrix4d m_WorldPose;
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
typedef  boost::shared_ptr<NodeMotionState>             NodeMotionStatePtr;

//////////////////////////////////////////////////////////
///
/// Entity class
/// Entity is a capsule containing the important info behind Rigid Bodies in the
/// ModelGraph.
///
//////////////////////////////////////////////////////////

class Entity
{
public:
  Entity(){
  }

  ~Entity(){
  }

  Entity(
      string sName,
      CollisionShapePtr  pShape, //< Input:
      NodeMotionStatePtr  pMotionState, //< Input:
      RigidBodyPtr pRigidBody //< Input:
      )
  {
    m_sName        = sName;
    m_pShape       = pShape;
    m_pMotionState = pMotionState;
    m_pRigidBody   = pRigidBody;
  }


  const char* GetParentName(){
    if (m_pMotionState->object.m_pParent) {
      return m_pMotionState->object.m_pParent->GetName().c_str();
    }
    return NULL;
  }

  string                  m_sName;
  CollisionShapePtr       m_pShape;
  NodeMotionStatePtr      m_pMotionState;
  RigidBodyPtr            m_pRigidBody;

};

#endif // PHYSICSENGINEHELPERS_H
