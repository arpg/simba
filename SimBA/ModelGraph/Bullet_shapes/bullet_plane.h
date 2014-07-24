#ifndef BULLET_PLANE_H
#define BULLET_PLANE_H

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include <ModelGraph/Bullet_shapes/bullet_shape.h>

//Constructs a Bullet btStaticPlaneShape.

class bullet_plane: public bullet_shape{

public:
  //constructor
  explicit bullet_plane(const std::shared_ptr<ModelNode>& mnPlane){
    PlaneShape* pPlane = (PlaneShape*) mnPlane.get();
    std::vector<double> dNormal = pPlane->m_dNormal;

    //Just make a flat plain
    bulletShape = new btStaticPlaneShape(
          btVector3(dNormal[0], dNormal[1], dNormal[2]), 0);
    bulletMotionState = new NodeMotionState(mnPlane);
    btRigidBody::btRigidBodyConstructionInfo cInfo(0, bulletMotionState,
                                                   bulletShape,
                                                   btVector3(0, 0, 0));
    bulletBody = new btRigidBody(cInfo);
  }

};

#endif // BULLET_PLANE_H
