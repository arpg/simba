#ifndef BULLET_CYLINDER_H
#define BULLET_CYLINDER_H

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btCylinderShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include "bullet_shape.h"

class bullet_cylinder : public bullet_shape{

public:
  //constructor
  bullet_cylinder(ModelNode* mnCylinder){
    CylinderShape* pCylinder = (CylinderShape*) mnCylinder;
    double dRadius = pCylinder->m_dRadius;
    double dHeight = pCylinder->m_dHeight;
    double dMass = pCylinder->GetMass();
    double dRestitution = pCylinder->GetRestitution();
    Eigen::Matrix4d dPose;
    dPose = pCylinder->GetPoseMatrix();

    bulletShape = new btCylinderShapeZ(btVector3(dRadius, dRadius, dHeight/2));
    bulletMotionState = new NodeMotionState( *mnCylinder );
    bool isDynamic = ( dMass != 0.f );
    btVector3 localInertia( 0, 0, 0 );
    if( isDynamic ){
        bulletShape->calculateLocalInertia( dMass, localInertia );
    }
    btRigidBody::btRigidBodyConstructionInfo  cInfo(dMass, bulletMotionState,
                                                    bulletShape, localInertia);
    bulletBody = new btRigidBody(cInfo);
    double dContactProcessingThreshold = 0.001;
    bulletBody->setContactProcessingThreshold( dContactProcessingThreshold );
    bulletBody->setRestitution( dRestitution );
    SetPose(dPose);
  }

};

#endif // BULLET_CYLINDER_H
