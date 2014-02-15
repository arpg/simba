#ifndef PHYSICSENGINE_H
#define PHYSICSENGINE_H

#include "PhysicsEngineHelpers.h"

//All of our Bullet Objects
//bullet_shape holds the header files Shapes.h and RaycastVehicle.h
#include <ModelGraph/Bullet_shapes/bullet_shape.h>
#include <ModelGraph/Bullet_shapes/bullet_cube.h>
#include <ModelGraph/Bullet_shapes/bullet_cylinder.h>
#include <ModelGraph/Bullet_shapes/bullet_sphere.h>
#include <ModelGraph/Bullet_shapes/bullet_vehicle.h>
#include <ModelGraph/Bullet_shapes/bullet_plane.h>

//////////////////////////////////////////////////////////
///
/// PhysicsEngine class
/// PhysicsEngine encapsulates all of the Physics engine (in this case, Bullet) into one
/// class. It initializes the physics environment, and allows for the addition
/// and deletion of objects. It must also be called to run the physics sim.
///
//////////////////////////////////////////////////////////


class PhysicsEngine
{

public:

  //////////////////////////////////////////////////////////
  ///
  /// CONSTRUCTOR
  ///
  //////////////////////////////////////////////////////////

  PhysicsEngine(){
    m_dTimeStep = 1.0/30.0;
    m_dGravity = 9.8;
    m_nMaxSubSteps = 10; // bullet -- for stepSimulation
  }

  bool Init(
      double dGravity = 9.8,
      double dTimeStep = 1.0/30.0,
      double nMaxSubSteps = 10
      ){
    m_dTimeStep    = dTimeStep;
    m_dGravity     = dGravity;
    m_nMaxSubSteps = nMaxSubSteps;
    // Physics stuff
    // See http://bulletphysics.org/mediawiki-1.5.8/index.php/Hello_World

    btDefaultCollisionConfiguration btCollConfig;
    m_CollisionConfiguration = &btCollConfig;
    m_pDispatcher = boost::shared_ptr<btCollisionDispatcher>(
          new btCollisionDispatcher(m_CollisionConfiguration));
    m_pBroadphase
        = boost::shared_ptr<btDbvtBroadphase>( new btDbvtBroadphase );
    m_pSolver
        = boost::shared_ptr<btSequentialImpulseConstraintSolver>(
          new btSequentialImpulseConstraintSolver );
    m_pDynamicsWorld = boost::shared_ptr<btDiscreteDynamicsWorld>(
          new btDiscreteDynamicsWorld(m_pDispatcher.get(),
                                      m_pBroadphase.get(),
                                      m_pSolver.get(),
                                      m_CollisionConfiguration)
          );

    m_pDynamicsWorld->setGravity( btVector3(0,0,m_dGravity) );
    m_pDynamicsWorld->setDebugDrawer( &m_DebugDrawer );
    m_pDynamicsWorld->getDebugDrawer()->
        setDebugMode(btIDebugDraw::DBG_DrawWireframe +
                     btIDebugDraw::DBG_FastWireframe +
                     btIDebugDraw::DBG_DrawConstraints);
    return true;
  }

  //////////////////////////////////////////////////////////
  ///
  /// ADDING OBJECTS TO THE PHYSICS ENGINE
  /// We got a really great selection going for ya.
  ///
  //////////////////////////////////////////////////////////

  void RegisterObject(ModelNode *pItem){

    /*********************************************************************
     *ADDING A RAYCAST VEHICLE
     **********************************************************************/
    if(dynamic_cast<RaycastVehicle*>(pItem)!=NULL){
      bullet_vehicle btRayVehicle( pItem, m_pDynamicsWorld.get());
      CollisionShapePtr pShape( btRayVehicle.getBulletShapePtr() );
      MotionStatePtr pMotionState( btRayVehicle.getBulletMotionStatePtr() );
      RigidBodyPtr body( btRayVehicle.getBulletBodyPtr() );
      VehiclePtr vehicle( btRayVehicle.getBulletRaycastVehicle() );
      boost::shared_ptr<Vehicle_Entity> pEntity( new Vehicle_Entity );
      pEntity->m_pRigidBody = body;
      pEntity->m_pShape = pShape;
      pEntity->m_pMotionState = pMotionState;
      pEntity->m_pVehicle = vehicle;
      int id = m_mRayVehicles.size();
      m_mRayVehicles[pItem->GetName()] = pEntity;
    }

    /*********************************************************************
     *ADDING SHAPES
     **********************************************************************/
    else if (dynamic_cast<Shape*>(pItem) != NULL) {
      Shape* pNodeShape = (Shape*) pItem;

      //Box
      if (dynamic_cast<BoxShape*>( pNodeShape ) != NULL) {
        bullet_cube btBox(pItem);
        CollisionShapePtr pShape( btBox.getBulletShapePtr() );
        MotionStatePtr pMotionState( btBox.getBulletMotionStatePtr() );
        RigidBodyPtr body( btBox.getBulletBodyPtr() );
        m_pDynamicsWorld->addRigidBody( body.get() );

        //Save the object; easier deconstruction this way.
        boost::shared_ptr<Entity> pEntity( new Entity );
        pEntity->m_pRigidBody = body;
        pEntity->m_pShape = pShape;
        pEntity->m_pMotionState = pMotionState;
        m_mShapes[pItem->GetName()] = pEntity;
      }

      //Cylinder
      else if (dynamic_cast<CylinderShape*>( pNodeShape ) != NULL) {
        bullet_cylinder btCylinder(pItem);
        CollisionShapePtr pShape( btCylinder.getBulletShapePtr() );
        MotionStatePtr pMotionState( btCylinder.getBulletMotionStatePtr() );
        RigidBodyPtr body( btCylinder.getBulletBodyPtr() );
        m_pDynamicsWorld->addRigidBody( body.get() );

        //Save the object; easier deconstruction this way.
        boost::shared_ptr<Entity> pEntity( new Entity );
        pEntity->m_pRigidBody = body;
        pEntity->m_pShape = pShape;
        pEntity->m_pMotionState = pMotionState;
        m_mShapes[pItem->GetName()] = pEntity;
      }

      //Plane
      else if (dynamic_cast<PlaneShape*>( pNodeShape ) != NULL) {
        bullet_plane btPlane(pItem);
        CollisionShapePtr pShape( btPlane.getBulletShapePtr() );
        MotionStatePtr pMotionState( btPlane.getBulletMotionStatePtr() );
        RigidBodyPtr body( btPlane.getBulletBodyPtr() );
        m_pDynamicsWorld->addRigidBody( body.get() );

        //Save the object; easier deconstruction this way.
        boost::shared_ptr<Entity> pEntity( new Entity );
        pEntity->m_pRigidBody = body;
        pEntity->m_pShape = pShape;
        pEntity->m_pMotionState = pMotionState;
        m_mShapes[pItem->GetName()] = pEntity;
      }

      //Mesh
      else if (dynamic_cast<MeshShape*>( pNodeShape ) != NULL){
        /// TODO:
        /// Write a bullet_mesh class.
        /// import through this framework.
      }

    }


    /*********************************************************************
     *ADDING CONSTRAINTS
     **********************************************************************/

    else if (dynamic_cast<Constraint*>(pItem) != NULL) {
      Constraint* pNodeCon = (Constraint*) pItem;
      // Point to Point
      if (dynamic_cast<PToPOne*>( pNodeCon ) != NULL) {
        PToPOne* pCon = (PToPOne*) pNodeCon;
        boost::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
        btVector3 pivot_A(pCon->m_pivot_in_A[0], pCon->m_pivot_in_A[1],
                          pCon->m_pivot_in_A[2]);
        btPoint2PointConstraint* PToP =
            new btPoint2PointConstraint(*Shape_A->m_pRigidBody.get(), pivot_A);
        m_pDynamicsWorld->addConstraint(PToP);
        m_mPtoP[pCon->GetName()] = PToP;
      }

      else if(dynamic_cast<PToPTwo*>( pNodeCon ) != NULL) {
        PToPTwo* pCon = (PToPTwo*) pNodeCon;
        boost::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
        boost::shared_ptr<Entity> Shape_B = m_mShapes.at(pCon->m_Shape_B);
        btVector3 pivot_A(pCon->m_pivot_in_A[0], pCon->m_pivot_in_A[1],
                          pCon->m_pivot_in_A[2]);
        btVector3 pivot_B(pCon->m_pivot_in_B[0], pCon->m_pivot_in_B[1],
                          pCon->m_pivot_in_B[2]);
        btPoint2PointConstraint* PToP =
            new btPoint2PointConstraint(*Shape_A->m_pRigidBody.get(),
                                        *Shape_B->m_pRigidBody.get(),
                                        pivot_A, pivot_B);
        m_pDynamicsWorld->addConstraint(PToP);
        m_mPtoP[pCon->GetName()] = PToP;
      }

      //Hinge
      else if(dynamic_cast<HingeOnePivot*>( pNodeCon ) != NULL) {
        HingeOnePivot* pCon = (HingeOnePivot*) pNodeCon;
        boost::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
        btVector3 pivot_A(pCon->m_pivot_in_A[0], pCon->m_pivot_in_A[1],
                          pCon->m_pivot_in_A[2]);
        btVector3 axis_A(pCon->m_axis_in_A[0], pCon->m_axis_in_A[1],
                         pCon->m_axis_in_A[2]);
        btHingeConstraint* Hinge =
            new btHingeConstraint(*Shape_A->m_pRigidBody.get(), pivot_A,
                                  axis_A, true);
        Hinge->setLimit(pCon->m_low_limit, pCon->m_high_limit, pCon->m_softness,
                        pCon->m_bias, pCon->m_relaxation);
        m_pDynamicsWorld->addConstraint(Hinge);
        m_mHinge[pCon->GetName()] = Hinge;
      }

      else if(dynamic_cast<HingeTwoPivot*>( pNodeCon ) != NULL) {
        HingeTwoPivot* pCon = (HingeTwoPivot*) pNodeCon;
        boost::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
        boost::shared_ptr<Entity> Shape_B = m_mShapes.at(pCon->m_Shape_B);
        btVector3 pivot_A(pCon->m_pivot_in_A[0], pCon->m_pivot_in_A[1],
                          pCon->m_pivot_in_A[2]);
        btVector3 axis_A(pCon->m_axis_in_A[0], pCon->m_axis_in_A[1],
                         pCon->m_axis_in_A[2]);
        btVector3 pivot_B(pCon->m_pivot_in_B[0], pCon->m_pivot_in_B[1],
                          pCon->m_pivot_in_B[2]);
        btVector3 axis_B(pCon->m_axis_in_B[0], pCon->m_axis_in_B[1],
                         pCon->m_axis_in_B[2]);
        btHingeConstraint* Hinge =
            new btHingeConstraint(*Shape_A->m_pRigidBody.get(),
                                  *Shape_B->m_pRigidBody.get(),
                                  pivot_A, pivot_B,
                                  axis_A, axis_B,
                                  true);
        Hinge->setLimit(pCon->m_low_limit, pCon->m_high_limit, pCon->m_softness,
                        pCon->m_bias, pCon->m_relaxation);
        m_pDynamicsWorld->addConstraint(Hinge);
        m_mHinge[pCon->GetName()] = Hinge;
      }

      //Hinge2
      else if(dynamic_cast<Hinge2*>( pNodeCon ) != NULL) {
        Hinge2* pCon = (Hinge2*) pNodeCon;
        boost::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
        boost::shared_ptr<Entity> Shape_B = m_mShapes.at(pCon->m_Shape_B);
        btVector3 btAnchor(pCon->m_Anchor[0], pCon->m_Anchor[1],
                           pCon->m_Anchor[2]);
        btVector3 btAxis_1(pCon->m_Axis_1[0], pCon->m_Axis_1[1],
                           pCon->m_Axis_1[2]);
        btVector3 btAxis_2(pCon->m_Axis_2[0], pCon->m_Axis_2[1],
                           pCon->m_Axis_2[2]);
        btHinge2Constraint* Hinge =
            new btHinge2Constraint(*Shape_A->m_pRigidBody.get(),
                                   *Shape_B->m_pRigidBody.get(),
                                   btAnchor, btAxis_1, btAxis_2);
        Hinge->setUpperLimit(pCon->m_steering_angle);
        Hinge->setLowerLimit(pCon->m_steering_angle);
        Hinge->enableSpring(3, true);
        Hinge->setStiffness(3, pCon->m_stiffness);
        Hinge->setDamping(3, pCon->m_damping);
        m_pDynamicsWorld->addConstraint(Hinge);
        m_mHinge2[pCon->GetName()] = Hinge;
      }
    }

    return;

  }

  ///////////////////////////////////////////////////////////////////

  void DebugDrawWorld(){
    m_pDynamicsWorld->debugDrawWorld();
  }

  ///////////////////////////////////////////////////////////////////

  /// TODO: Fix whatever's up with the m_pDynamicsWorld...\
  ///
  ///
  ///

  void StepSimulation(){
    m_pDynamicsWorld->stepSimulation( m_dTimeStep,  m_nMaxSubSteps );
  }

  //////////////////////////////////////////////////////////
  ///
  /// PRINT FUNCTIONS
  ///
  //////////////////////////////////////////////////////////

  void PrintAllShapes(){
    for(std::map<string, boost::shared_ptr<Entity> > ::iterator it =m_mShapes.begin(); it!=m_mShapes.end(); it++ ){
      std::cout<<it->first<<std::endl;
    }
  }

  //////////////////////////////////////////////////////////
  ///
  /// OBJECT DELETION
  ///
  //////////////////////////////////////////////////////////
  void DeleteHingeConstraintFromDynamicsWorld(string sConstraintName){
    btHingeConstraint* h = getHingeConstraint(sConstraintName);
    btDiscreteDynamicsWorld* dynmaicsWorld = m_pDynamicsWorld.get();
    dynmaicsWorld->removeConstraint(h);
  }

  ///////////////////////////////////////////////////////////////////
  void DeleteHinge2ConstraintFromDynamicsWorld(string sConstraintName){
    btHinge2Constraint* h = getHinge2Constraint(sConstraintName);
    btDiscreteDynamicsWorld* dynmaicsWorld = m_pDynamicsWorld.get();
    dynmaicsWorld->removeConstraint(h);
  }

  ///////////////////////////////////////////////////////////////////
  // Important! Must remove the constraint before removing the rigid Shape
  void DeleteRigidBodyFromDynamicsWorld(string sShapeFullName){
    Entity e = getEntity(sShapeFullName);
    btRigidBody* objectShape = e.m_pRigidBody.get();

    btDiscreteDynamicsWorld* dynmaicsWorld = m_pDynamicsWorld.get();

    for(int i = dynmaicsWorld->getNumCollisionObjects()-1;i>=0;i--){
      btCollisionObject* obj =dynmaicsWorld->getCollisionObjectArray()[i];
      btRigidBody* Shape = btRigidBody::upcast(obj);

      if(Shape == objectShape){
        if (Shape && Shape->getMotionState()){
          delete Shape->getMotionState();
        }
        dynmaicsWorld->removeCollisionObject(obj);
        delete obj;
      }
    }
  }

  ///////////////////////////////////////////////////////////////////
  // Remove all bodies and joints in m_mShapes mapping
  void EraseEntityInShapeList(string sEntityName){
    boost::shared_ptr<Entity> pEntity =  m_mShapes.find(sEntityName)->second;
    pEntity.reset();
    cout<<"find entity "<<sEntityName<< " success"<<endl;
  }

  //////////////////////////////////////////////////////////
  ///
  /// GETTERS
  ///
  //////////////////////////////////////////////////////////
  //  boost::shared_ptr<Entity> GetParentEntity( Entity &e )
  //  {
  //    if (e.GetParentName()) {
  //      return m_mShapes[e.GetParentName()];
  //    }
  //    return boost::shared_ptr<Entity>();
  //  }

  //  ///////////////////////////////////////////////////////////////////
  //  Eigen::Matrix4d GetRelativePose( Entity &rChild)
  //  {
  //    boost::shared_ptr<Entity> pParent = GetParentEntity(rChild);
  //    if (pParent) {
  //      btTransform btTwp;
  //      btTransform btTwc;
  //      pParent->m_pMotionState->getWorldTransform(btTwp);
  //      rChild.m_pMotionState->getWorldTransform(btTwc);
  //      Eigen::Matrix4d Twp = toEigen(btTwp);
  //      Eigen::Matrix4d Twc = toEigen(btTwc);
  //      return getInverseTransformation(Twp)*Twc; /// find Tinv in mvl or something
  //    }
  //    return Eigen::Matrix4d::Identity();
  //  }

  ///////////////////////////////////////////////////////////////////
  Entity getEntity(string name)
  {
    if(m_mShapes.find(name) !=m_mShapes.end())
    {
      Entity e =*m_mShapes.find(name)->second;
      return e;
    }
    else
    {
      cout<<"[PhysicsEngine] Fatal Error! Cannot get entity '"<<name<<"'. Exit!"<<endl;
      vector<string> Names = GetAllEntityName();
      for(int ii = 0; ii<Names.size(); ii++){
        cout<<Names.at(ii)<<endl;
      }
      Entity e =*m_mShapes.find(name)->second;
      return e;
    }
  }

  ///////////////////////////////////////////////////////////////////
  vector<string> GetAllEntityName(){
    vector<string> vNameList;
    std::map<string, boost::shared_ptr<Entity> >::iterator iter = m_mShapes.begin();
    for(iter = m_mShapes.begin();iter!=m_mShapes.end(); iter++){
      string sFullName = iter->first;
      vNameList.push_back(sFullName);
    }
    return vNameList;
  }


  ///////////////////////////////////////////////////////////////////

  btHinge2Constraint* getHinge2Constraint(string name){
    std::map<string, btHinge2Constraint*>::iterator iter = m_mHinge2.find(name);
    if(iter!=m_mHinge2.end()){
      btHinge2Constraint* pHJ2 = m_mHinge2.find(name)->second;
      return pHJ2;
    }
    else{
      cout<<"Fatal Error! Cannot get Hinge2joint "<<name<<endl;
      btHinge2Constraint* pHJ2 = m_mHinge2.find(name)->second;
      return pHJ2;
    }
  }

  ///////////////////////////////////////////////////////////////////

  btHingeConstraint* getHingeConstraint(string name){
    std::map<string, btHingeConstraint*>::iterator iter = m_mHinge.find(name);
    if(iter!=m_mHinge.end()){
      btHingeConstraint* pHJ = iter->second;
      return pHJ;
    }
    else{
      cout<<"Fatal Error! Cannot get Hingejoint "<<name<<endl;
      btHingeConstraint* pHJ = iter->second;
      return pHJ;
    }
  }

  ///////////////////////////////////////////////////////////////////

  btDynamicsWorld* GetDynamicsWorld(){
    btDynamicsWorld* pDynmaicsWorld= m_pDynamicsWorld.get();
    return pDynmaicsWorld;
  }

  /*******************************************************
    *
    * ENTITY FUNCTIONS
    *
    ******************************************************/

  Eigen::Vector6d GetEntity6Pose( string name )
  {
    Entity e = getEntity( name );
    btRigidBody* rB = e.m_pRigidBody.get();

    btTransform WorldTransform = rB->getCenterOfMassTransform();

    btScalar roll, pitch, yaw;
    WorldTransform.getBasis().getEulerZYX(yaw,pitch,roll);
    double x, y, z, r, p, q;
    x = WorldTransform.getOrigin().getX();
    y = WorldTransform.getOrigin().getY();
    z = WorldTransform.getOrigin().getZ();
    r = roll;
    p = pitch;
    q = yaw;
    Eigen::Vector6d toRet;
    toRet << x, y, z, r, p, q;

    return toRet;
  }

  ///////////////////////////////////////////////////////////////////

  void GetEntity6Pose(string name, Eigen::Vector6d& rPose)
  {
    Entity e = getEntity(name);
    btRigidBody* rB = e.m_pRigidBody.get();

    btTransform WorldTransform = rB->getCenterOfMassTransform();

    btScalar roll, pitch, yaw;
    WorldTransform.getBasis().getEulerZYX(yaw,pitch,roll);

    rPose[0] = WorldTransform.getOrigin().getX();
    rPose[1] = WorldTransform.getOrigin().getY();
    rPose[2] = WorldTransform.getOrigin().getZ();
    rPose[3] = roll;
    rPose[4] = pitch;
    rPose[5] = yaw;
  }

  ///////////////////////////////////////////////////////////////////

  void SetEntity6Pose(string sName, Eigen::Vector6d Pose)
  {
    SetEntityRotation(sName, Pose(3,0), Pose(4,0), Pose(5,0));

    Eigen::Vector3d eOrigin;
    eOrigin<<Pose(0,0), Pose(1,0), Pose(2,0);
    SetEntityOrigin(sName, eOrigin);
  }

  ///////////////////////////////////////////////////////////////////

  Eigen::Vector3d GetEntityOrigin(string sName)
  {

    Entity e = getEntity(sName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btVector3 btOrigin = rB->getCenterOfMassTransform().getOrigin();

    Eigen::Vector3d eOrigin;
    eOrigin<< btOrigin.getX(), btOrigin.getY() , btOrigin.getZ();

    return eOrigin;
  }

  ///////////////////////////////////////////////////////////////////

  Eigen::Matrix3d GetEntityBasis(string sName)
  {
    Entity e = getEntity(sName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btMatrix3x3 btBasis = rB->getCenterOfMassTransform().getBasis();

    Eigen::Matrix3d mBasis;
    mBasis << btBasis[0][0], btBasis[0][1], btBasis[0][2],
        btBasis[1][0], btBasis[1][1], btBasis[1][2],
        btBasis[2][0], btBasis[2][1], btBasis[2][2];
    return mBasis;
  }

  ///////////////////////////////////////////////////////////////////

  void SetEntityOrigin(string sName, Eigen::Vector3d eOrigin)
  {
    Entity e = getEntity(sName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btTransform btTran = rB->getCenterOfMassTransform();

    btVector3 BtOrigin = btTran.getOrigin();
    BtOrigin.setX(eOrigin[0]);
    BtOrigin.setY(eOrigin[1]);
    BtOrigin.setZ(eOrigin[2]);
    btTran.setOrigin(BtOrigin);

    rB->setCenterOfMassTransform(btTran);
  }

  ///////////////////////////////////////////////////////////////////

  void SetEntityBasis(string sName, Eigen::Matrix3d mBasis)
  {
    Entity e = getEntity(sName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btTransform btTran = rB->getCenterOfMassTransform();

    btMatrix3x3 btBasis;
    btBasis.setValue(mBasis(0,0),mBasis(0,1),mBasis(0,2),mBasis(1,0),mBasis(1,1),mBasis(1,2),mBasis(2,0),mBasis(2,1),mBasis(2,2));
    btTran.setBasis(btBasis);

    rB->setCenterOfMassTransform(btTran);
  }

  ///////////////////////////////////////////////////////////////////

  Eigen::Vector3d GetEntityLinearVelocity(string sBodyFullName)
  {
    Entity e = getEntity(sBodyFullName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btVector3 LinVelocity = rB->getLinearVelocity();

    Eigen::Vector3d  eLinearVelocity;
    eLinearVelocity<<LinVelocity.getX(),LinVelocity.getY(),LinVelocity.getZ();

    return eLinearVelocity;
  }

  ///////////////////////////////////////////////////////////////////

  double GetEntityVelocity(string name)
  {
    Entity e = getEntity(name);
    btRigidBody* rB = e.m_pRigidBody.get();

    double vx = rB->getLinearVelocity()[0];
    double vy = rB->getLinearVelocity()[1];
    double vz = rB->getLinearVelocity()[2];

    double velocity = sqrt(vx*vx + vy*vy+vz*vz);
    return velocity;
  }

  ///////////////////////////////////////////////////////////////////

  Eigen::Vector3d GetEntityAngularVelocity(string sBodyFullName)
  {
    Entity e = getEntity(sBodyFullName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btVector3 AngVelocity = rB->getAngularVelocity();

    Eigen::Vector3d  eAngularVelocity;
    eAngularVelocity<<AngVelocity.getX(),AngVelocity.getY(),AngVelocity.getZ();

    return eAngularVelocity;
  }

  ///////////////////////////////////////////////////////////////////

  void SetEntityLinearvelocity(string sBodyFullName, Eigen::Vector3d eLinearVelocity)
  {
    Entity e = getEntity(sBodyFullName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btVector3 btLinearvelocity;
    btLinearvelocity.setX(eLinearVelocity[0]);
    btLinearvelocity.setY(eLinearVelocity[1]);
    btLinearvelocity.setZ(eLinearVelocity[2]);

    rB->setLinearVelocity(btLinearvelocity);
  }

  ///////////////////////////////////////////////////////////////////

  void SetEntityAngularvelocity(string sBodyFullName, Eigen::Vector3d eAngularVelocity)
  {
    Entity e = getEntity(sBodyFullName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btVector3 btAngularvelocity;
    btAngularvelocity.setX(eAngularVelocity[0]);
    btAngularvelocity.setY(eAngularVelocity[1]);
    btAngularvelocity.setZ(eAngularVelocity[2]);

    rB->setAngularVelocity(btAngularvelocity);
  }

  ///////////////////////////////////////////////////////////////////

  void SetEntityRotation(string EntityName, double roll, double pitch, double yaw)
  {
    Entity e = getEntity(EntityName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btTransform tr= rB->getCenterOfMassTransform();
    btQuaternion quat;
    cout<<"try to set roll "<<roll<<" pitch "<<pitch<<" yaw "<<yaw<<endl;
    quat.setEulerZYX(yaw,pitch,roll);
    tr.setRotation(quat);

    rB->setCenterOfMassTransform(tr);
    PrintEntityRotation(EntityName);
  }

  ///////////////////////////////////////////////////////////////////

  void GetEntityRotation(string EntityName, double& roll, double& pitch, double& yaw)
  {
    Entity e = getEntity(EntityName);
    btRigidBody* rB = e.m_pRigidBody.get();
    btQuaternion quat = rB->getCenterOfMassTransform().getRotation();
    roll = quat.getX();
    pitch = quat.getY();
    yaw = quat.getZ();
  }

  ///////////////////////////////////////////////////////////////////

  void PrintEntityRotation(string EntityName)
  {
    Entity e = getEntity(EntityName);
    btRigidBody* rB = e.m_pRigidBody.get();
    btQuaternion quat = rB->getCenterOfMassTransform().getRotation();
    double roll = quat.getX();
    double pitch = quat.getY();
    double yaw = quat.getZ();
    cout<<"get roll "<<roll<<" pitch "<<pitch<<" yaw "<<yaw<<endl;
  }

  ///////////////////////////////////////////////////////////////////
  /// force, steering and Torque

  void SetFriction(string name, double F)
  {
    Entity e = getEntity(name);
    btRigidBody* rB = e.m_pRigidBody.get(); //.m_pRigidBody->

    rB->setFriction(F);
  }

  ///////////////////////////////////////////////////////////////////

  void ApplyForceToEntity(string name, double F)
  {
    Entity e = getEntity(name);
    btRigidBody* rB = e.m_pRigidBody.get(); //.m_pRigidBody->

    btVector3 force;
    force.setX(F);
    force.setY(0);
    force.setZ(0);

    rB->applyCentralForce(force);
  }

  ///////////////////////////////////////////////////////////////////

  void ApplyTorque(string sBodyFullName, Eigen::Vector3d eTorque)
  {
    Entity e = getEntity(sBodyFullName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btVector3 Torque;

    Torque.setX(eTorque[0]);
    Torque.setY(eTorque[1]);
    Torque.setZ(eTorque[2]);

    Torque = (rB->getCenterOfMassTransform().getBasis())*Torque;

    rB->applyTorque(Torque);
  }

  ///////////////////////////////////////////////////////////////////

  void ApplySteering(string sBodyFullName, Eigen::Vector3d eSteering)
  {
    Entity Wheel = getEntity(sBodyFullName);

    btRigidBody* pWheel = Wheel.m_pRigidBody.get();
    btVector3 eSteer;
    eSteer[0] = eSteering[0];
    eSteer[1] = eSteering[1];
    eSteer[2] = eSteering[2];
    pWheel->applyTorque(eSteer);
  }

  ///////////////////////////////////////////////////////////////////
  /// serialize and deserialize. experimental .. not working now
  // return the serialize char* buffer for rigid body with 'bodyname'

  void SerializeRigidBodyToChar(string sBodyName, const unsigned char*& pData, int& iDataSize)
  {
    Entity e = getEntity(sBodyName);
    btRigidBody* rB = e.m_pRigidBody.get();

    int maxSerializeBufferSize = 1024*1024*5;
    btDefaultSerializer serializer (maxSerializeBufferSize);

    serializer.startSerialization();
    serializer.registerNameForPointer(rB,"fuck!!!!");
    rB->serializeSingleObject(&serializer);
    serializer.finishSerialization();

    pData = serializer.getBufferPointer();
    iDataSize = serializer.getCurrentBufferSize();
  }

  //////////////////////////////////////////////////////////
  ///
  /// MEMBER VARIABLES
  ///
  //////////////////////////////////////////////////////////

  DebugDraw                                               m_DebugDrawer;
  std::map<string, boost::shared_ptr<Vehicle_Entity> >    m_mRayVehicles;
  std::map<string, boost::shared_ptr<Entity> >            m_mShapes;
  std::map<string, boost::shared_ptr<Compound_Entity> >   m_mCompounds;
  std::map<string, btHingeConstraint*>                    m_mHinge;  // map of all hinge constraints
  std::map<string, btHinge2Constraint*>                   m_mHinge2; // map of all hinge 2 constraints
  std::map<string, btGeneric6DofConstraint*>              m_mSixDOF;
  std::map<string, btPoint2PointConstraint*>              m_mPtoP;
  boost::shared_ptr<btDiscreteDynamicsWorld>              m_pDynamicsWorld;

private:

  btDefaultCollisionConfiguration*                       m_CollisionConfiguration;
  boost::shared_ptr<btCollisionDispatcher>               m_pDispatcher;
  boost::shared_ptr<btDbvtBroadphase>                    m_pBroadphase;
  boost::shared_ptr<btSequentialImpulseConstraintSolver> m_pSolver;
  double                                                 m_dTimeStep;
  double                                                 m_dGravity;
  int                                                    m_nMaxSubSteps;

};


#endif // PHYSICSENGINE_H
