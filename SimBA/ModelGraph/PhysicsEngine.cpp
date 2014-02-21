#include "PhysicsEngine.h"

  //////////////////////////////////////////////////////////
  ///
  /// CONSTRUCTOR
  ///
  //////////////////////////////////////////////////////////

  PhysicsEngine::PhysicsEngine(){
    m_dTimeStep = 1.0/30.0;
    m_dGravity = -9.8;
    m_nMaxSubSteps = 10; // bullet -- for stepSimulation
  }

  bool PhysicsEngine::Init(double dGravity, double dTimeStep,
                           double nMaxSubSteps){
    m_dTimeStep    = dTimeStep;
    m_dGravity     = dGravity;
    m_nMaxSubSteps = nMaxSubSteps;


    // Physics stuff
    // See http://bulletphysics.org/mediawiki-1.5.8/index.php/Hello_World

    m_pDispatcher
        = boost::shared_ptr<btCollisionDispatcher>(
          new btCollisionDispatcher(&m_CollisionConfiguration) );
    m_pBroadphase
        = boost::shared_ptr<btDbvtBroadphase>( new btDbvtBroadphase );
    m_pSolver
        = boost::shared_ptr<btSequentialImpulseConstraintSolver>(
          new btSequentialImpulseConstraintSolver );
    m_pDynamicsWorld = boost::shared_ptr<btDiscreteDynamicsWorld>(
          new btDiscreteDynamicsWorld(m_pDispatcher.get(),
                                      m_pBroadphase.get(),
                                      m_pSolver.get(),
                                      &m_CollisionConfiguration
                                      ) );
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

  void PhysicsEngine::RegisterObject(ModelNode *pItem){

    /*********************************************************************
     *ADDING SHAPES
     **********************************************************************/
    if (dynamic_cast<Shape*>(pItem) != NULL) {
      Shape* pNodeShape = (Shape*) pItem;

      /************************************
       *ADDING A RAYCAST VEHICLE
       ************************************/

      if(dynamic_cast<RaycastVehicle*>(pNodeShape)!=NULL){
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
        m_mRayVehicles[pItem->GetName()] = pEntity;
      }

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
        btRigidBody* RigidShape_A;
        if(isVehicle(pCon->m_Shape_A)){
          boost::shared_ptr<Vehicle_Entity> Shape_A = m_mRayVehicles.at(pCon->m_Shape_A);
          RigidShape_A = Shape_A->m_pRigidBody.get();
        }
        else{
          boost::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
          RigidShape_A = Shape_A->m_pRigidBody.get();
        }
        btVector3 pivot_A(pCon->m_pivot_in_A[0], pCon->m_pivot_in_A[1],
                          pCon->m_pivot_in_A[2]);
        btPoint2PointConstraint* PToP =
            new btPoint2PointConstraint(*RigidShape_A, pivot_A);
        m_pDynamicsWorld->addConstraint(PToP);
        m_mPtoP[pCon->GetName()] = PToP;
      }

      else if(dynamic_cast<PToPTwo*>( pNodeCon ) != NULL) {
        PToPTwo* pCon = (PToPTwo*) pNodeCon;
        btRigidBody* RigidShape_A;
        btRigidBody* RigidShape_B;
        if(isVehicle(pCon->m_Shape_A)){
          boost::shared_ptr<Vehicle_Entity> Shape_A = m_mRayVehicles.at(pCon->m_Shape_A);
          RigidShape_A = Shape_A->m_pRigidBody.get();
        }
        else{
          boost::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
          RigidShape_A = Shape_A->m_pRigidBody.get();
        }
        if(isVehicle(pCon->m_Shape_B)){
          boost::shared_ptr<Vehicle_Entity> Shape_B = m_mRayVehicles.at(pCon->m_Shape_B);
          RigidShape_B = Shape_B->m_pRigidBody.get();
        }
        else{
          boost::shared_ptr<Entity> Shape_B = m_mShapes.at(pCon->m_Shape_B);
          RigidShape_B = Shape_B->m_pRigidBody.get();
        }
        btVector3 pivot_A(pCon->m_pivot_in_A[0], pCon->m_pivot_in_A[1],
                          pCon->m_pivot_in_A[2]);
        btVector3 pivot_B(pCon->m_pivot_in_B[0], pCon->m_pivot_in_B[1],
                          pCon->m_pivot_in_B[2]);
        btPoint2PointConstraint* PToP =
            new btPoint2PointConstraint(*RigidShape_A, *RigidShape_B,
                                        pivot_A, pivot_B);
        m_pDynamicsWorld->addConstraint(PToP);
        m_mPtoP[pCon->GetName()] = PToP;
      }

      //Hinge
      else if(dynamic_cast<HingeOnePivot*>( pNodeCon ) != NULL) {
        HingeOnePivot* pCon = (HingeOnePivot*) pNodeCon;
        btRigidBody* RigidShape_A;
        if(isVehicle(pCon->m_Shape_A)){
          boost::shared_ptr<Vehicle_Entity> Shape_A = m_mRayVehicles.at(pCon->m_Shape_A);
          RigidShape_A = Shape_A->m_pRigidBody.get();
        }
        else{
          boost::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
          RigidShape_A = Shape_A->m_pRigidBody.get();
        }
        btVector3 pivot_A(pCon->m_pivot_in_A[0], pCon->m_pivot_in_A[1],
                          pCon->m_pivot_in_A[2]);
        btVector3 axis_A(pCon->m_axis_in_A[0], pCon->m_axis_in_A[1],
                         pCon->m_axis_in_A[2]);
        btHingeConstraint* Hinge =
            new btHingeConstraint(*RigidShape_A, pivot_A,
                                  axis_A, true);
        Hinge->setLimit(pCon->m_low_limit, pCon->m_high_limit, pCon->m_softness,
                        pCon->m_bias, pCon->m_relaxation);
        m_pDynamicsWorld->addConstraint(Hinge);
        m_mHinge[pCon->GetName()] = Hinge;
      }

      else if(dynamic_cast<HingeTwoPivot*>( pNodeCon ) != NULL) {
        HingeTwoPivot* pCon = (HingeTwoPivot*) pNodeCon;
        btRigidBody* RigidShape_A;
        btRigidBody* RigidShape_B;
        if(isVehicle(pCon->m_Shape_A)){
          boost::shared_ptr<Vehicle_Entity> Shape_A = m_mRayVehicles.at(pCon->m_Shape_A);
          RigidShape_A = Shape_A->m_pRigidBody.get();
        }
        else{
          boost::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
          RigidShape_A = Shape_A->m_pRigidBody.get();
        }
        if(isVehicle(pCon->m_Shape_B)){
          boost::shared_ptr<Vehicle_Entity> Shape_B = m_mRayVehicles.at(pCon->m_Shape_B);
          RigidShape_B = Shape_B->m_pRigidBody.get();
        }
        else{
          boost::shared_ptr<Entity> Shape_B = m_mShapes.at(pCon->m_Shape_B);
          RigidShape_B = Shape_B->m_pRigidBody.get();
        }
        btVector3 pivot_A(pCon->m_pivot_in_A[0], pCon->m_pivot_in_A[1],
                          pCon->m_pivot_in_A[2]);
        btVector3 axis_A(pCon->m_axis_in_A[0], pCon->m_axis_in_A[1],
                         pCon->m_axis_in_A[2]);
        btVector3 pivot_B(pCon->m_pivot_in_B[0], pCon->m_pivot_in_B[1],
                          pCon->m_pivot_in_B[2]);
        btVector3 axis_B(pCon->m_axis_in_B[0], pCon->m_axis_in_B[1],
                         pCon->m_axis_in_B[2]);
        btHingeConstraint* Hinge =
            new btHingeConstraint(*RigidShape_A,
                                  *RigidShape_B,
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
        btRigidBody* RigidShape_A;
        btRigidBody* RigidShape_B;
        if(isVehicle(pCon->m_Shape_A)){
          boost::shared_ptr<Vehicle_Entity> Shape_A = m_mRayVehicles.at(pCon->m_Shape_A);
          RigidShape_A = Shape_A->m_pRigidBody.get();
        }
        else{
          boost::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
          RigidShape_A = Shape_A->m_pRigidBody.get();
        }
        if(isVehicle(pCon->m_Shape_B)){
          boost::shared_ptr<Vehicle_Entity> Shape_B = m_mRayVehicles.at(pCon->m_Shape_B);
          RigidShape_B = Shape_B->m_pRigidBody.get();
        }
        else{
          boost::shared_ptr<Entity> Shape_B = m_mShapes.at(pCon->m_Shape_B);
          RigidShape_B = Shape_B->m_pRigidBody.get();
        }
        btVector3 btAnchor(pCon->m_Anchor[0], pCon->m_Anchor[1],
                           pCon->m_Anchor[2]);
        btVector3 btAxis_1(pCon->m_Axis_1[0], pCon->m_Axis_1[1],
                           pCon->m_Axis_1[2]);
        btVector3 btAxis_2(pCon->m_Axis_2[0], pCon->m_Axis_2[1],
                           pCon->m_Axis_2[2]);
        btHinge2Constraint* Hinge =
            new btHinge2Constraint(*RigidShape_A, *RigidShape_B,
                                   btAnchor, btAxis_1, btAxis_2);
        Hinge->setAngularLowerLimit(toBulletVec3(pCon->m_LowerAngLimit));
        Hinge->setAngularUpperLimit(toBulletVec3(pCon->m_UpperAngLimit));
        Hinge->setLinearLowerLimit(toBulletVec3(pCon->m_LowerLinLimit));
        Hinge->setLinearUpperLimit(toBulletVec3(pCon->m_UpperLinLimit));
        Hinge->enableSpring(3, true);
        Hinge->setStiffness(3, pCon->m_stiffness);
        Hinge->setDamping(3, pCon->m_damping);
        m_pDynamicsWorld->addConstraint(Hinge);
        m_mHinge2[pCon->GetName()] = Hinge;
      }

      //SixDOF
      else if(dynamic_cast<SixDOFOne*>( pNodeCon ) != NULL) {
        SixDOFOne* pCon = (SixDOFOne*) pNodeCon;
        boost::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
        btTransform trans_A = toBullet(_Cart2T(pCon->m_Transform_A));
        btGeneric6DofConstraint* SixDOF =
            new btGeneric6DofConstraint(*Shape_A->m_pRigidBody.get(),
                                        trans_A,
                                        true);
        SixDOF->setLinearLowerLimit(toBulletVec3(pCon->m_LowerLinLimit));
        SixDOF->setLinearUpperLimit(toBulletVec3(pCon->m_UpperLinLimit));
        SixDOF->setAngularLowerLimit(toBulletVec3(pCon->m_LowerAngLimit));
        SixDOF->setAngularUpperLimit(toBulletVec3(pCon->m_UpperAngLimit));
        m_pDynamicsWorld->addConstraint(SixDOF);
        m_mSixDOF[pCon->GetName()] = SixDOF;
      }

      else if(dynamic_cast<SixDOFTwo*>( pNodeCon ) != NULL) {
        SixDOFTwo* pCon = (SixDOFTwo*) pNodeCon;
        btRigidBody* RigidShape_A;
        btRigidBody* RigidShape_B;
        if(isVehicle(pCon->m_Shape_A)){
          boost::shared_ptr<Vehicle_Entity> Shape_A = m_mRayVehicles.at(pCon->m_Shape_A);
          RigidShape_A = Shape_A->m_pRigidBody.get();
        }
        else{
          boost::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
          RigidShape_A = Shape_A->m_pRigidBody.get();
        }
        if(isVehicle(pCon->m_Shape_B)){
          boost::shared_ptr<Vehicle_Entity> Shape_B = m_mRayVehicles.at(pCon->m_Shape_B);
          RigidShape_B = Shape_B->m_pRigidBody.get();
        }
        else{
          boost::shared_ptr<Entity> Shape_B = m_mShapes.at(pCon->m_Shape_B);
          RigidShape_B = Shape_B->m_pRigidBody.get();
        }
        btTransform trans_A = toBullet(_Cart2T(pCon->m_Transform_A));
        btTransform trans_B = toBullet(_Cart2T(pCon->m_Transform_B));
        btGeneric6DofConstraint* SixDOF =
            new btGeneric6DofConstraint(*RigidShape_A, *RigidShape_B,
                                        trans_A, trans_B,
                                        true);
        SixDOF->setLinearLowerLimit(toBulletVec3(pCon->m_LowerLinLimit));
        SixDOF->setLinearUpperLimit(toBulletVec3(pCon->m_UpperLinLimit));
        SixDOF->setAngularLowerLimit(toBulletVec3(pCon->m_LowerAngLimit));
        SixDOF->setAngularUpperLimit(toBulletVec3(pCon->m_UpperAngLimit));
        m_pDynamicsWorld->addConstraint(SixDOF);
        m_mSixDOF[pCon->GetName()] = SixDOF;
      }
    }

    return;

  }

  bool PhysicsEngine::isVehicle(string Shape){
    bool vehi = false;
    std::size_t found = Shape.find("RaycastVehicle");
      if (found!=std::string::npos){
        vehi = true;
      }
      return vehi;
  }

  /***********************************************
    *
    * RUNNING THE SIMULATION
    *
    **********************************************/

  void PhysicsEngine::DebugDrawWorld(){
    m_pDynamicsWorld->debugDrawWorld();
  }

  ////////////////////////////////

  void PhysicsEngine::StepSimulation(){
    m_pDynamicsWorld->stepSimulation( m_dTimeStep,  m_nMaxSubSteps );
    if(m_mRayVehicles.size()!=0){
      // Go through all of our vehicles and update their part poses.
      for(std::map<string, boost::shared_ptr<Vehicle_Entity> > ::iterator it =
          m_mRayVehicles.begin(); it!=m_mRayVehicles.end(); it++ ){
        // The MotionStatePointer holds the ModelNode, which holds the poses.
        Vehicle_Entity* eVehicle = it->second.get();
        NodeMotionState* mMotion = eVehicle->m_pMotionState.get();
        RaycastVehicle* pVehicle = (RaycastVehicle*) &mMotion->object;
        std::vector<Eigen::Matrix4d> VehiclePoses =
            GetVehicleTransform(pVehicle->GetName());
        pVehicle->SetPose(_T2Cart(VehiclePoses.at(0)));
        pVehicle->SetWheelPose(0, _T2Cart(VehiclePoses.at(1)));
        pVehicle->SetWheelPose(1, _T2Cart(VehiclePoses.at(2)));
        pVehicle->SetWheelPose(2, _T2Cart(VehiclePoses.at(3)));
        pVehicle->SetWheelPose(3, _T2Cart(VehiclePoses.at(4)));
      }
    }
  }

  //////////////////////////////////////////////////////////
  ///
  /// PRINT FUNCTIONS
  ///
  //////////////////////////////////////////////////////////

  void PhysicsEngine::PrintAllShapes(){
    for(std::map<string, boost::shared_ptr<Entity> > ::iterator it =
        m_mShapes.begin(); it!=m_mShapes.end(); it++ ){
      std::cout<<it->first<<std::endl;
    }
  }

  //////////////////////////////////////////////////////////
  ///
  /// OBJECT DELETION
  ///
  //////////////////////////////////////////////////////////
  void PhysicsEngine::DeleteHingeConstraintFromDynamicsWorld(string sConstraintName){
    btHingeConstraint* h = getHingeConstraint(sConstraintName);
    btDiscreteDynamicsWorld* dynmaicsWorld = m_pDynamicsWorld.get();
    dynmaicsWorld->removeConstraint(h);
  }

  ///////////////////////////////////////////////////////////////////
  void PhysicsEngine::DeleteHinge2ConstraintFromDynamicsWorld(string sConstraintName){
    btHinge2Constraint* h = getHinge2Constraint(sConstraintName);
    btDiscreteDynamicsWorld* dynmaicsWorld = m_pDynamicsWorld.get();
    dynmaicsWorld->removeConstraint(h);
  }

  ///////////////////////////////////////////////////////////////////
  // Important! Must remove the constraint before removing the rigid Shape
  void PhysicsEngine::DeleteRigidBodyFromDynamicsWorld(string sShapeFullName){
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
  void PhysicsEngine::EraseEntityInShapeList(string sEntityName){
    boost::shared_ptr<Entity> pEntity =  m_mShapes.find(sEntityName)->second;
    pEntity.reset();
    cout<<"find entity "<<sEntityName<< " success"<<endl;
  }

  //////////////////////////////////////////////////////////
  ///
  /// GETTERS
  ///
  //////////////////////////////////////////////////////////

  Entity PhysicsEngine::getEntity(string name){
    if(m_mShapes.find(name) !=m_mShapes.end()){
      Entity e =*m_mShapes.find(name)->second;
      return e;
    }
    else{
      cout<<"[PhysicsEngine] Fatal Error! Cannot get entity '"<<
            name<<"'. Exit!"<<endl;
      vector<string> Names = GetAllEntityName();
      for(unsigned int ii = 0; ii<Names.size(); ii++){
        cout<<Names.at(ii)<<endl;
      }
      Entity e =*m_mShapes.find(name)->second;
      return e;
    }
  }

  ///////////////////////////////////////////////////////////////////
  vector<string> PhysicsEngine::GetAllEntityName(){
    vector<string> vNameList;
    std::map<string, boost::shared_ptr<Entity> >::iterator iter =
        m_mShapes.begin();
    for(iter = m_mShapes.begin();iter!=m_mShapes.end(); iter++){
      string sFullName = iter->first;
      vNameList.push_back(sFullName);
    }
    return vNameList;
  }

  ///////////////////////////////////////////////////////////////////

  btHinge2Constraint* PhysicsEngine::getHinge2Constraint(string name){
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

  btHingeConstraint* PhysicsEngine::getHingeConstraint(string name){
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

  btDynamicsWorld* PhysicsEngine::GetDynamicsWorld(){
    btDynamicsWorld* pDynmaicsWorld= m_pDynamicsWorld.get();
    return pDynmaicsWorld;
  }

  ///////////////////////////////////////////////////////////////////

  Eigen::Vector6d PhysicsEngine::GetEntity6Pose( string name )
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

  void PhysicsEngine::GetEntity6Pose(string name, Eigen::Vector6d& rPose)
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

  void PhysicsEngine::SetEntity6Pose(string sName, Eigen::Vector6d Pose)
  {
    SetEntityRotation(sName, Pose(3,0), Pose(4,0), Pose(5,0));

    Eigen::Vector3d eOrigin;
    eOrigin<<Pose(0,0), Pose(1,0), Pose(2,0);
    SetEntityOrigin(sName, eOrigin);
  }

  ///////////////////////////////////////////////////////////////////

  Eigen::Vector3d PhysicsEngine::GetEntityOrigin(string sName)
  {

    Entity e = getEntity(sName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btVector3 btOrigin = rB->getCenterOfMassTransform().getOrigin();

    Eigen::Vector3d eOrigin;
    eOrigin<< btOrigin.getX(), btOrigin.getY() , btOrigin.getZ();

    return eOrigin;
  }

  ///////////////////////////////////////////////////////////////////

  Eigen::Matrix3d PhysicsEngine::GetEntityBasis(string sName)
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

  void PhysicsEngine::SetEntityOrigin(string sName, Eigen::Vector3d eOrigin)
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

  void PhysicsEngine::SetEntityBasis(string sName, Eigen::Matrix3d mBasis)
  {
    Entity e = getEntity(sName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btTransform btTran = rB->getCenterOfMassTransform();

    btMatrix3x3 btBasis;
    btBasis.setValue(mBasis(0,0),mBasis(0,1),mBasis(0,2),
                     mBasis(1,0),mBasis(1,1),mBasis(1,2),
                     mBasis(2,0),mBasis(2,1),mBasis(2,2));
    btTran.setBasis(btBasis);

    rB->setCenterOfMassTransform(btTran);
  }

  ///////////////////////////////////////////////////////////////////

  Eigen::Vector3d PhysicsEngine::GetEntityLinearVelocity(string sBodyFullName)
  {
    Entity e = getEntity(sBodyFullName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btVector3 LinVelocity = rB->getLinearVelocity();

    Eigen::Vector3d  eLinearVelocity;
    eLinearVelocity<<LinVelocity.getX(),LinVelocity.getY(),LinVelocity.getZ();

    return eLinearVelocity;
  }

  ///////////////////////////////////////////////////////////////////

  double PhysicsEngine::GetEntityVelocity(string name)
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

  Eigen::Vector3d PhysicsEngine::GetEntityAngularVelocity(string sBodyFullName){
    Entity e = getEntity(sBodyFullName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btVector3 AngVelocity = rB->getAngularVelocity();

    Eigen::Vector3d  eAngularVelocity;
    eAngularVelocity<<AngVelocity.getX(),AngVelocity.getY(),AngVelocity.getZ();

    return eAngularVelocity;
  }

  ///////////////////////////////////////////////////////////////////

  void PhysicsEngine::SetEntityLinearvelocity(string sBodyFullName,
                               Eigen::Vector3d eLinearVelocity){
    Entity e = getEntity(sBodyFullName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btVector3 btLinearvelocity;
    btLinearvelocity.setX(eLinearVelocity[0]);
    btLinearvelocity.setY(eLinearVelocity[1]);
    btLinearvelocity.setZ(eLinearVelocity[2]);

    rB->setLinearVelocity(btLinearvelocity);
  }

  ///////////////////////////////////////////////////////////////////

  void PhysicsEngine::SetEntityAngularvelocity(string sBodyFullName,
                                Eigen::Vector3d eAngularVelocity){
    Entity e = getEntity(sBodyFullName);
    btRigidBody* rB = e.m_pRigidBody.get();

    btVector3 btAngularvelocity;
    btAngularvelocity.setX(eAngularVelocity[0]);
    btAngularvelocity.setY(eAngularVelocity[1]);
    btAngularvelocity.setZ(eAngularVelocity[2]);

    rB->setAngularVelocity(btAngularvelocity);
  }

  ///////////////////////////////////////////////////////////////////

  void PhysicsEngine::SetEntityRotation(string EntityName, double roll, double pitch,
                         double yaw){
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

  void PhysicsEngine::GetEntityRotation(string EntityName, double& roll, double& pitch,
                         double& yaw)
  {
    Entity e = getEntity(EntityName);
    btRigidBody* rB = e.m_pRigidBody.get();
    btQuaternion quat = rB->getCenterOfMassTransform().getRotation();
    roll = quat.getX();
    pitch = quat.getY();
    yaw = quat.getZ();
  }

  ///////////////////////////////////////////////////////////////////

  void PhysicsEngine::PrintEntityRotation(string EntityName){
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

  void PhysicsEngine::SetFriction(string name, double F)
  {
    Entity e = getEntity(name);
    btRigidBody* rB = e.m_pRigidBody.get(); //.m_pRigidBody->

    rB->setFriction(F);
  }

  ///////////////////////////////////////////////////////////////////

  void PhysicsEngine::ApplyForceToEntity(string name, double F){
    Entity e = getEntity(name);
    btRigidBody* rB = e.m_pRigidBody.get(); //.m_pRigidBody->

    btVector3 force;
    force.setX(F);
    force.setY(0);
    force.setZ(0);

    rB->applyCentralForce(force);
  }

  ///////////////////////////////////////////////////////////////////

  void PhysicsEngine::ApplyTorque(string sBodyFullName, Eigen::Vector3d eTorque){
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

  void PhysicsEngine::ApplySteering(string sBodyFullName, Eigen::Vector3d eSteering){
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

  void PhysicsEngine::SerializeRigidBodyToChar(string sBodyName, const unsigned char*& pData,
                                int& iDataSize){
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
  /// VEHICLE POSE GETTERS
  ///
  //////////////////////////////////////////////////////////

  std::vector< Eigen::Matrix4d > PhysicsEngine::GetVehiclePoses( Vehicle_Entity* Vehicle ){
    std::vector<Eigen::Matrix4d> VehiclePoses;
    btTransform VehiclePose;
    VehiclePose.setIdentity();
    VehiclePose = Vehicle->m_pVehicle->getChassisWorldTransform();
    VehiclePoses.push_back(toEigen(VehiclePose));
    for( int i = 0; i<Vehicle->m_pVehicle->getNumWheels(); i++){
      Vehicle->m_pVehicle->updateWheelTransform(i,false);
      btTransform WheelPose;
      WheelPose.setIdentity();
      WheelPose = Vehicle->m_pVehicle->getWheelTransformWS(i);
      VehiclePoses.push_back(toEigen(WheelPose));
    }
    return VehiclePoses;
  }

  // Call GetVehicleTransform in RenderEngine whenever there's a RaycastVehicle
  // to render; since we use five GLObjects for one Bullet object, we have
  // to perform some trickery.

  std::vector<Eigen::Matrix4d> PhysicsEngine::GetVehicleTransform(std::string sVehicleName){
    boost::shared_ptr<Vehicle_Entity> boost_Vehicle =
        m_mRayVehicles.at(sVehicleName);
    Vehicle_Entity* Vehicle = boost_Vehicle.get();
    std::vector<Eigen::Matrix4d> Eig_transforms = GetVehiclePoses(Vehicle);
    return Eig_transforms;
  }
