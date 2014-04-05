#ifndef BULLET_HEIGHTMAP_H
#define BULLET_HEIGHTMAP_H

#include "ModelGraph/Bullet_shapes/bullet_shape.h"
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <bullet/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h>
#include <bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.h>
#include <bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>

//Constructs a Bullet btHeightfieldTerrainShape.

class bullet_heightmap{

public:
  //constructor
  bullet_heightmap(ModelNode* mnMap){
    HeightmapShape* pMap = (HeightmapShape*) mnMap;
    int row_count = pMap->m_nRowCount;
    int col_count = pMap->m_nColCount;
    const double* X = pMap->m_dXData;
    const double* Y = pMap->m_dYData;
    const double* Z = pMap->m_dZData;

    //    if(max_ht<=1){
    //      //Just make a flat plain
    //      bulletShape = new btStaticPlaneShape(
    //            btVector3(normal[0], normal[1], normal[2]), 0);
    //      bulletMotionState = new btDefaultMotionState(btTransform::getIdentity());
    //      btRigidBody::btRigidBodyConstructionInfo cInfo(0, bulletMotionState,
    //                                                     bulletShape,
    //                                                     btVector3(0, 0, 0));
    //      bulletBody = new btRigidBody(cInfo);
    //    }
    //    else{
    //////////////
    //Algorithm for populating BVHTriangleMeshShape taken from VehicleDemo.cpp

    int vertStride = sizeof(btVector3);
    int indexStride = 3*sizeof(int);
    const int NUM_VERTS_X = row_count;
    const int NUM_VERTS_Y = col_count;
    const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
    const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);
    btVector3* m_vertices = new btVector3[totalVerts];
    int* gIndices = new int[totalTriangles*3];
    for (int i=0;i<NUM_VERTS_X;i++){
      for (int j=0;j<NUM_VERTS_Y;j++){
        double width = X[i+j*NUM_VERTS_X];
        double length = Y[i+j*NUM_VERTS_X];
        double height = Z[i+j*NUM_VERTS_X];
        m_vertices[i+j*NUM_VERTS_X].setValue(width, length, height);
      }
    }

    int index=0;
    for (int i=0;i<NUM_VERTS_X-1;i++)
    {
      for (int j=0;j<NUM_VERTS_Y-1;j++)
      {
        gIndices[index++] = j*NUM_VERTS_X+i;
        gIndices[index++] = j*NUM_VERTS_X+i+1;
        gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

        gIndices[index++] = j*NUM_VERTS_X+i;
        gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
        gIndices[index++] = (j+1)*NUM_VERTS_X+i;
      }
    }

    btTriangleIndexVertexArray* m_indexVertexArrays =
        new btTriangleIndexVertexArray(totalTriangles,
                                       gIndices,
                                       indexStride,
                                       totalVerts,
                                       (btScalar*) &m_vertices[0].x(),
        vertStride);
    /////////////
    bulletShape = new btBvhTriangleMeshShape(m_indexVertexArrays, true);
    bulletMotionState = new NodeMotionState(*mnMap);
    btRigidBody::btRigidBodyConstructionInfo cInfo(0, bulletMotionState,
                                                   bulletShape,
                                                   btVector3(0, 0, 0));
    bulletBody = new btRigidBody(cInfo);
  }
  //  }

  //////////////////////////

  ///getters
  btCollisionShape* getBulletShapePtr(){
    return bulletShape;
  }

  btRigidBody* getBulletBodyPtr(){
    return bulletBody;
  }

  NodeMotionState* getBulletMotionStatePtr(){
    return bulletMotionState;
  }

private:
  btCollisionShape* bulletShape;
  btRigidBody* bulletBody;
  NodeMotionState* bulletMotionState;

};




#endif // BULLET_HEIGHTMAP_H
