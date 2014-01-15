#ifndef PHYSICSENGINE_H
#define PHYSICSENGINE_H

#include "PhysicsEngineHelpers.h"

//////////////////////////////////////////////////////////
///
/// Phys class
/// Phys encapsulates all of the Physics engine (in this case, Bullet) into one
/// class. It initializes the physics environment, and allows for the addition
/// and deletion of objects. It must also be called to run the physics sim.
///
//////////////////////////////////////////////////////////


class Phys
{

public:

  //////////////////////////////////////////////////////////
  ///
  /// CONSTRUCTOR
  ///
  //////////////////////////////////////////////////////////

  Phys(){
    m_dTimeStep = 1.0/30.0;
    m_dGravity = 9.8;
    m_nMaxSubSteps = 10; // bullet -- for stepSimulation
  }

  void Init(
      double dGravity = 9.8,       //< Input:
      double dTimeStep = 1.0/60.0, //< Input:
      double nMaxSubSteps = 1     //< Input: for stepSimulation
      ){
    m_dTimeStep    = dTimeStep;
    m_dGravity     = dGravity;
    m_nMaxSubSteps = nMaxSubSteps;

    // Physics stuff
    // See http://bulletphysics.org/mediawiki-1.5.8/index.php/Hello_World

    btCollisionDispatcher* ptr =
        new btCollisionDispatcher(&m_CollisionConfiguration);
    m_pDispatcher = boost::shared_ptr<btCollisionDispatcher>(ptr);
    m_pBroadphase
        = boost::shared_ptr<btDbvtBroadphase>( new btDbvtBroadphase );
    m_pSolver
        = boost::shared_ptr<btSequentialImpulseConstraintSolver>(
          new btSequentialImpulseConstraintSolver );
    m_pDynamicsWorld = boost::shared_ptr<btDiscreteDynamicsWorld>(
          new btDiscreteDynamicsWorld(m_pDispatcher.get(),
                                      m_pBroadphase.get(),
                                      m_pSolver.get(),
                                      &m_CollisionConfiguration)
          );
    m_pDynamicsWorld->setGravity( btVector3(0,0,m_dGravity) );
    m_pDynamicsWorld->setDebugDrawer( &m_DebugDrawer );
    m_pDynamicsWorld->getDebugDrawer()->
        setDebugMode(btIDebugDraw::DBG_DrawWireframe +
                     btIDebugDraw::DBG_FastWireframe +
                     btIDebugDraw::DBG_DrawConstraints);
  }

  //////////////////////////////////////////////////////////
  ///
  /// SIMULATOR FUNCTIONS
  ///
  //////////////////////////////////////////////////////////
  void RegisterObject(ModelNode *pItem, string sName,
                      Eigen::Vector6d WorldPose,
                      double dDefaultRestitution = 0){
    btVector3 localInertia( 0, 0, 0 );
    double dMass = 0.0f;

    /// Add Bullet Bodies (Box, Cylinder, etc...)

    if (dynamic_cast<Body*>(pItem) != NULL) {
      Body* pNodeBody = (Body*) pItem;
      if (dynamic_cast<BoxShape*>( pNodeBody->m_CollisionShape ) != NULL) {
        BoxShape* pCollisionShape = (BoxShape*) pNodeBody->m_CollisionShape;
        //                    std::cout<<"Associating physics with a BOX!"<<std::endl;
        dMass = pNodeBody->m_dMass;
        btVector3 bounds = toBulletVec3( pCollisionShape->m_dDims );
        CollisionShapePtr pBulletShape( new btBoxShape(bounds) );
        btAssert((!pBulletShape || pBulletShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));
        bool isDynamic = ( dMass != 0.f );
        if( isDynamic ){
          pBulletShape->calculateLocalInertia( dMass, localInertia );
        }
        NodeMotionStatePtr pMotionState( new NodeMotionState(*pNodeBody, WorldPose) );
        btRigidBody::btRigidBodyConstructionInfo  cInfo( dMass, pMotionState.get(), pBulletShape.get(), localInertia );
        RigidBodyPtr  pBulletBody( new btRigidBody(cInfo) );
        double dDefaultContactProcessingThreshold = 0.001;
        pBulletBody->setContactProcessingThreshold( dDefaultContactProcessingThreshold );
        pBulletBody->setRestitution( dDefaultRestitution );
        m_pDynamicsWorld->addRigidBody( pBulletBody.get() );

        // save this object somewhere (to keep it's reference count above 0)
        boost::shared_ptr<Entity> pEntity( new Entity );
        pEntity->m_sName = sName;
        pEntity->m_pRigidBody.swap(pBulletBody);
        pEntity->m_pShape.swap(pBulletShape);
        pEntity->m_pMotionState.swap(pMotionState);

        int id = m_mEntities.size();
        m_mEntities[sName] = pEntity;
      }

      else if (dynamic_cast<CylinderShape*>( pNodeBody->m_CollisionShape ) != NULL) {
        CylinderShape* pCollisionShape = (CylinderShape*) pNodeBody->m_CollisionShape;
        //                    std::cout<<"Associating physics with a CYLINDER!"<<std::endl;
        dMass = pNodeBody->m_dMass;
        btVector3 bounds = toBulletVec3( pCollisionShape->m_dRadius, 0.5*pCollisionShape->m_dThickness, pCollisionShape->m_dRadius );
        CollisionShapePtr pBulletShape( new btCylinderShape(bounds) );
        btAssert((!pBulletShape || pBulletShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));
        bool isDynamic = ( dMass != 0.f );
        if( isDynamic ){
          pBulletShape->calculateLocalInertia( dMass, localInertia );
        }

        NodeMotionStatePtr pMotionState( new NodeMotionState(*pNodeBody, WorldPose) );
        btRigidBody::btRigidBodyConstructionInfo  cInfo( dMass, pMotionState.get(), pBulletShape.get(), localInertia );
        RigidBodyPtr  pBulletBody( new btRigidBody(cInfo) );

        double dDefaultContactProcessingThreshold = 0.001;
        pBulletBody->setContactProcessingThreshold( dDefaultContactProcessingThreshold );
        pBulletBody->setRestitution( dDefaultRestitution );

        m_pDynamicsWorld->addRigidBody( pBulletBody.get() );

        // save this object somewhere (to keep it's reference count above 0)
        boost::shared_ptr<Entity> pEntity( new Entity );
        pEntity->m_sName = sName;
        pEntity->m_pRigidBody.swap(pBulletBody);
        pEntity->m_pShape.swap(pBulletShape);
        pEntity->m_pMotionState.swap(pMotionState);

        int id = m_mEntities.size();
        m_mEntities[sName] = pEntity;
      }
    }

    /// Add Bullet Joints (Hinge, Hinge2...)

    else if (dynamic_cast<HingeJoint*>(pItem) != NULL) {
      //                std::cout<<"Associating physics with a Hinge Joint!"<<std::endl;
      HingeJoint* pHJ = (HingeJoint*) pItem;

      boost::shared_ptr<Entity> pParent = m_mEntities[pHJ->m_pParentBody->GetName()];
      btRigidBody* pBodyA =  pParent->m_pRigidBody.get(); //localCreateRigidBody( 1.0f, tr, shape);

      boost::shared_ptr<Entity> pChild = m_mEntities[pHJ->m_pChildBody->GetName()];
      btRigidBody* pBodyB = pChild->m_pRigidBody.get();

      pBodyA->setActivationState(DISABLE_DEACTIVATION);
      pBodyB->setActivationState(DISABLE_DEACTIVATION);

      // add some data to build constraint frames
      Eigen::Vector6d v = pHJ->GetPose();
      Eigen::Vector3d v3;

      v3 = pHJ->m_dAxis1;
      btVector3 axisA = toBulletVec3( v3 );   // The axisA is the hinge axis direction in the A ref frame
      btVector3 axisB = toBulletVec3( v3 );   // The axisB is the hinge axis direction in the B ref frame

      if (dynamic_cast<CylinderShape*>(pHJ->m_pChildBody->m_CollisionShape)){
        axisB = toBulletVec3(pHJ->m_dAxis2);
      }

      v3 = pHJ->m_dPivot1;
      btVector3 pivotA = toBulletVec3( v3 );  // The pivotA is the pivoting point in the A frame

      // Now convert the pivot axis in A to the pivot axis in B [pB = pA - vAB]
      Eigen::Vector6d vAB6 = pHJ->m_pChildBody->GetPose();
      v3[0] -= vAB6[0];
      v3[1] -= vAB6[1];
      v3[2] -= vAB6[2];
      btVector3 pivotB = toBulletVec3( v3 );  // The pivotB is the pivoting in the B frame

      btHingeConstraint* spHingeDynAB = new btHingeConstraint(*pBodyA, *pBodyB, pivotA, pivotB, axisA, axisB);
      double dLowerLimit = pHJ->m_dLowerLimit;
      double dUpperLimit = pHJ->m_dUpperLimit;
      spHingeDynAB->setLimit(dLowerLimit,dUpperLimit);

      // add constraint to world
      m_pDynamicsWorld->addConstraint(spHingeDynAB, true);

      // save pointer to hinge contraint
      string sHingeName = pHJ->GetName();
      m_mHingeJointList.insert(std::pair<std::string,btHingeConstraint*>(sHingeName,spHingeDynAB));
    }

    else if (dynamic_cast<Hinge2Joint*>(pItem) != NULL) {
      //                std::cout<<"Associating physics with a Hinge2Joint!"<<std::endl;
      Hinge2Joint* pH2J = (Hinge2Joint*) pItem;

      boost::shared_ptr<Entity> pParent = m_mEntities[pH2J->m_pParentBody->GetName()];
      btRigidBody* pBodyA =  pParent->m_pRigidBody.get(); //localCreateRigidBody( 1.0f, tr, shape);


      boost::shared_ptr<Entity> pChild = m_mEntities[pH2J->m_pChildBody->GetName()];
      btRigidBody* pBodyB = pChild->m_pRigidBody.get();

      pBodyA->setActivationState(DISABLE_DEACTIVATION);
      pBodyB->setActivationState(DISABLE_DEACTIVATION);

      btVector3 axis1 = toBulletVec3( pH2J->m_dAxis1 );   // The axisA is the hinge axis direction in the A ref frame
      btVector3 axis2 = toBulletVec3( pH2J->m_dAxis2 );   // The axisB is the hinge axis direction in the B ref frame
      btVector3 anchor = toBulletVec3( pH2J->m_dAnchor ); // The anchor point in world coordinates

      btHinge2Constraint* spHingeDynAB = new btHinge2Constraint(*pBodyA, *pBodyB, anchor, axis1, axis2);
      spHingeDynAB->setLinearLowerLimit(toBulletVec3(pH2J->m_dLowerLimit));
      spHingeDynAB->setLinearUpperLimit(toBulletVec3(pH2J->m_dUpperLimit));
      spHingeDynAB->setAngularLowerLimit(toBulletVec3(pH2J->m_dLowerALimit));
      spHingeDynAB->setAngularUpperLimit(toBulletVec3(pH2J->m_dUpperALimit));
      spHingeDynAB->setDamping(0, pH2J->m_dDamping);

      // add constraint to world
      m_pDynamicsWorld->addConstraint(spHingeDynAB, true);

      // draw constraint frames and limits for debugging
      spHingeDynAB->setDbgDrawSize(btScalar(5.f));

      // save pointer to hinge2 contraint
      string sHingeName = pH2J->GetName();
      m_mHinge2JointList.insert(std::pair<std::string,btHinge2Constraint*>(sHingeName,spHingeDynAB));
    }

    return;

  }

  ///////////////////////////////////////////////////////////////////

  void DebugDrawWorld(){
    m_pDynamicsWorld->debugDrawWorld();
  }

  ///////////////////////////////////////////////////////////////////

  void StepSimulation(){
    m_pDynamicsWorld->stepSimulation( m_dTimeStep,  m_nMaxSubSteps );
  }

  //////////////////////////////////////////////////////////
  ///
  /// PRINT FUNCTIONS
  ///
  //////////////////////////////////////////////////////////

  void PrintAllConstraintName(){
    std::map<string, btHingeConstraint*>::iterator iter = m_mHingeJointList.begin();
    for(;iter!=m_mHingeJointList.end();iter++){
      cout<<"get hinge constraint name "<<iter->first<<endl;
    }
    std::map<string, btHinge2Constraint*>::iterator iter1 = m_mHinge2JointList.begin();
    for(;iter1!=m_mHinge2JointList.end();iter1++){
      cout<<"get hinge 2 constraint name "<<iter1->first<<endl;
    }
  }

  ///////////////////////////////////////////////////////////////////

  void PrintAllEntityName(){
    std::map<string, boost::shared_ptr<Entity> >::iterator iter = m_mEntities.begin();
    for(iter = m_mEntities.begin();iter!=m_mEntities.end(); iter++){
      string sFullName = iter->first;
      cout<<"Get current Entity Full Name "<<sFullName<<endl;
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

  // Important! Must remove the constraint before removing the rigid body

  void DeleteRigidbodyFromDynamicsWorld(string sBodyFullName){
    Entity e = getEntity(sBodyFullName);
    btRigidBody* objectbody = e.m_pRigidBody.get();

    btDiscreteDynamicsWorld* dynmaicsWorld = m_pDynamicsWorld.get();

    for(int i = dynmaicsWorld->getNumCollisionObjects()-1;i>=0;i--){
      btCollisionObject* obj =dynmaicsWorld->getCollisionObjectArray()[i];
      btRigidBody* body = btRigidBody::upcast(obj);

      if(body == objectbody){
        if (body && body->getMotionState()){
          delete body->getMotionState();
        }
        dynmaicsWorld->removeCollisionObject(obj);
        delete obj;
      }
    }
  }

  ///////////////////////////////////////////////////////////////////

  // Remove all bodies and joints in m_mEntities mapping

  void EraseBodyInEntitiesList(string sEntityName){
    boost::shared_ptr<Entity> pEntity =  m_mEntities.find(sEntityName)->second;
    pEntity.reset();
    cout<<"find entity "<<sEntityName<< " success"<<endl;
  }

  //////////////////////////////////////////////////////////
  ///
  /// GETTERS
  ///
  //////////////////////////////////////////////////////////

  boost::shared_ptr<Entity> GetParentEntity( Entity &e )
  {
    if (e.GetParentName()) {
      return m_mEntities[e.GetParentName()];
    }
    return boost::shared_ptr<Entity>();
  }

  ///////////////////////////////////////////////////////////////////

  Eigen::Matrix4d GetRelativePose( Entity &rChild)
  {
    boost::shared_ptr<Entity> pParent = GetParentEntity(rChild);
    if (pParent) {
      Eigen::Matrix4d& Twp = pParent->m_pMotionState->m_WorldPose;
      Eigen::Matrix4d& Twc = rChild.m_pMotionState->m_WorldPose;
      return getInverseTransformation(Twp)*Twc; /// find Tinv in mvl or something
    }
    return Eigen::Matrix4d::Identity();
  }

  ///////////////////////////////////////////////////////////////////

  Entity getEntity(string name){
    if(m_mEntities.find(name) !=m_mEntities.end()){
      Entity e =*m_mEntities.find(name)->second;
      return e;
    }
    else{
      cout<<"Fatal Error! Cannot get entity '"<<name<<"'. Exit!"<<endl;
      Entity e =*m_mEntities.find(name)->second;
      return e;
    }
  }

  ///////////////////////////////////////////////////////////////////

  vector<string> GetAllEntityName(){
    vector<string> vNameList;
    std::map<string, boost::shared_ptr<Entity> >::iterator iter = m_mEntities.begin();
    for(iter = m_mEntities.begin();iter!=m_mEntities.end(); iter++){
      string sFullName = iter->first;
      vNameList.push_back(sFullName);
    }
    return vNameList;
  }

  ///////////////////////////////////////////////////////////////////

  btHinge2Constraint* getHinge2Constraint(string name){
    std::map<string, btHinge2Constraint*>::iterator iter = m_mHinge2JointList.find(name);
    if(iter!=m_mHinge2JointList.end()){
      btHinge2Constraint* pHJ2 = m_mHinge2JointList.find(name)->second;
      return pHJ2;
    }
    else{
      cout<<"Fatal Error! Cannot get Hinge2joint "<<name<<endl;
      btHinge2Constraint* pHJ2 = m_mHinge2JointList.find(name)->second;
      return pHJ2;
    }
  }

  ///////////////////////////////////////////////////////////////////

  btHingeConstraint* getHingeConstraint(string name){
    std::map<string, btHingeConstraint*>::iterator iter = m_mHingeJointList.find(name);
    if(iter!=m_mHingeJointList.end()){
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

  //////////////////////////////////////////////////////////
  ///
  /// MEMBER VARIABLES
  ///
  //////////////////////////////////////////////////////////

  RigidBodyPtr                                           m_carBody;
  DebugDraw                                              m_DebugDrawer;
  std::map<string, boost::shared_ptr<Entity> >           m_mEntities;        // map of all rigid bodys
  std::map<string, btHingeConstraint*>                   m_mHingeJointList;  // map of all hinge constraints
  std::map<string, btHinge2Constraint*>                  m_mHinge2JointList; // map of all hinge 2 constraints
  boost::shared_ptr<btDiscreteDynamicsWorld>             m_pDynamicsWorld;

private:

  btDefaultCollisionConfiguration                        m_CollisionConfiguration;
  boost::shared_ptr<btCollisionDispatcher>               m_pDispatcher;
  boost::shared_ptr<btDbvtBroadphase>                    m_pBroadphase;
  boost::shared_ptr<btSequentialImpulseConstraintSolver> m_pSolver;
  double                                                 m_dTimeStep;
  double                                                 m_dGravity;
  int                                                    m_nMaxSubSteps;

};


#endif // PHYSICSENGINE_H
