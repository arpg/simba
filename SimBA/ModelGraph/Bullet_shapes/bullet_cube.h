#ifndef BULLET_CUBE_H
#define BULLET_CUBE_H

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btBoxShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include "bullet_shape.h"

class bullet_cube : public bullet_shape {
 public:
  // constructor
  explicit bullet_cube(const std::shared_ptr<ModelNode>& mnBox) {
    BoxShape* pBox = (BoxShape*) mnBox.get();
    double x_length = pBox->m_dBounds[0];
    double y_length = pBox->m_dBounds[1];
    double z_length = pBox->m_dBounds[2];
    double dMass = pBox->GetMass();
    double dRestitution = pBox->GetRestitution();
    Eigen::Matrix4d dPose;
    dPose = pBox->GetPoseMatrix();

    btVector3 bounds = btVector3(x_length*.5, y_length*.5, z_length*.5);
    bulletShape = new btBoxShape(bounds);
    bulletMotionState = new NodeMotionState(mnBox);
    bool isDynamic = (dMass != 0.f);
    btVector3 localInertia(0, 0, 0);
    if (isDynamic) {
      bulletShape->calculateLocalInertia(dMass, localInertia);
    }
    btRigidBody::btRigidBodyConstructionInfo  cInfo(dMass, bulletMotionState,
                                                    bulletShape, localInertia);
    bulletBody = new btRigidBody(cInfo);
    bulletBody->setRestitution(dRestitution);
    SetPose(dPose);
  }
};


#endif //  BULLET_CUBE_H
