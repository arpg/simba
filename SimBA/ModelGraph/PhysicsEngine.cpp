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


  std::shared_ptr<btCollisionDispatcher> Dispatcher(
        new btCollisionDispatcher(&m_CollisionConfiguration) );
  m_pDispatcher = Dispatcher;

  std::shared_ptr<btDbvtBroadphase> Broadphase( new btDbvtBroadphase );
  m_pBroadphase = Broadphase;

  std::shared_ptr<btSequentialImpulseConstraintSolver> Solver(
        new btSequentialImpulseConstraintSolver );
  m_pSolver = Solver;
  std::shared_ptr<btDiscreteDynamicsWorld> DWorld(
        new btDiscreteDynamicsWorld(m_pDispatcher.get(),
                                    m_pBroadphase.get(),
                                    m_pSolver.get(),
                                    &m_CollisionConfiguration
                                    ) );
  m_pDynamicsWorld = DWorld;
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

    if(dynamic_cast<SimRaycastVehicle*>(pNodeShape)!=NULL){
      bullet_vehicle btRayVehicle( pItem, m_pDynamicsWorld.get());
      CollisionShapePtr pShape( btRayVehicle.getBulletShapePtr() );
      MotionStatePtr pMotionState( btRayVehicle.getBulletMotionStatePtr() );
      RigidBodyPtr body( btRayVehicle.getBulletBodyPtr() );
      VehiclePtr vehicle( btRayVehicle.getBulletRaycastVehicle() );
      std::shared_ptr<Vehicle_Entity> pEntity( new Vehicle_Entity );
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
      std::shared_ptr<Entity> pEntity( new Entity );
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
      std::shared_ptr<Entity> pEntity( new Entity );
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
      std::shared_ptr<Entity> pEntity( new Entity );
      pEntity->m_pRigidBody = body;
      pEntity->m_pShape = pShape;
      pEntity->m_pMotionState = pMotionState;
      m_mShapes[pItem->GetName()] = pEntity;
    }

    //Heightmap
    else if (dynamic_cast<HeightmapShape*>( pNodeShape ) != NULL) {
      bullet_heightmap btMap(pItem);
      CollisionShapePtr pShape( btMap.getBulletShapePtr() );
      MotionStatePtr pMotionState( btMap.getBulletMotionStatePtr() );
      RigidBodyPtr body( btMap.getBulletBodyPtr() );
      m_pDynamicsWorld->addRigidBody( body.get() );

      //Save the object; easier deconstruction this way.
      std::shared_ptr<Entity> pEntity( new Entity );
      pEntity->m_pRigidBody = body;
      pEntity->m_pShape = pShape;
      pEntity->m_pMotionState = pMotionState;
      m_mShapes[pItem->GetName()] = pEntity;
    }

    //Mesh
    else if (dynamic_cast<MeshShape*>( pNodeShape ) != NULL){
      bullet_mesh btMesh(pItem);
      CollisionShapePtr pShape( btMesh.getBulletShapePtr() );
      MotionStatePtr pMotionState( btMesh.getBulletMotionStatePtr() );
      RigidBodyPtr body( btMesh.getBulletBodyPtr() );
      m_pDynamicsWorld->addRigidBody( body.get() );

      //Save the object; easier deconstruction this way.
      std::shared_ptr<Entity> pEntity( new Entity );
      pEntity->m_pRigidBody = body;
      pEntity->m_pShape = pShape;
      pEntity->m_pMotionState = pMotionState;
      m_mShapes[pItem->GetName()] = pEntity;
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
        std::shared_ptr<Vehicle_Entity> Shape_A = m_mRayVehicles.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
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
        std::shared_ptr<Vehicle_Entity> Shape_A = m_mRayVehicles.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      if(isVehicle(pCon->m_Shape_B)){
        std::shared_ptr<Vehicle_Entity> Shape_B = m_mRayVehicles.at(pCon->m_Shape_B);
        RigidShape_B = Shape_B->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_B = m_mShapes.at(pCon->m_Shape_B);
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
        std::shared_ptr<Vehicle_Entity> Shape_A = m_mRayVehicles.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
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
        std::shared_ptr<Vehicle_Entity> Shape_A = m_mRayVehicles.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      if(isVehicle(pCon->m_Shape_B)){
        std::shared_ptr<Vehicle_Entity> Shape_B = m_mRayVehicles.at(pCon->m_Shape_B);
        RigidShape_B = Shape_B->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_B = m_mShapes.at(pCon->m_Shape_B);
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
        std::shared_ptr<Vehicle_Entity> Shape_A = m_mRayVehicles.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      if(isVehicle(pCon->m_Shape_B)){
        std::shared_ptr<Vehicle_Entity> Shape_B = m_mRayVehicles.at(pCon->m_Shape_B);
        RigidShape_B = Shape_B->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_B = m_mShapes.at(pCon->m_Shape_B);
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
      std::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
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
        std::shared_ptr<Vehicle_Entity> Shape_A = m_mRayVehicles.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_A = m_mShapes.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      if(isVehicle(pCon->m_Shape_B)){
        std::shared_ptr<Vehicle_Entity> Shape_B = m_mRayVehicles.at(pCon->m_Shape_B);
        RigidShape_B = Shape_B->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_B = m_mShapes.at(pCon->m_Shape_B);
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

///////////////////////////////////////////////////////

void PhysicsEngine::RegisterDevice(SimDeviceInfo* pDevice){
  m_mDevices.push_back(pDevice);
}

///////////////////////////////////////////////////////

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


void PhysicsEngine::RunDevices(){
  for(unsigned int ii=0; ii<m_mDevices.size(); ii++){
    ///////////////
    // CAR CONTROLLER
    ///////////////
    SimDeviceInfo* Device = m_mDevices.at(ii);
    if(Device->m_sDeviceType=="CarController"){
      // We have to check all controllers.
      // TODO: Better way: do this mapping as soon as we add the controller.
      CarController* pCarCon = (CarController*) m_mDevices.at(ii);
      for(std::map<string, std::shared_ptr<Vehicle_Entity> > ::iterator it =
          m_mRayVehicles.begin(); it!=m_mRayVehicles.end(); it++ ){
        if(pCarCon->GetBodyName()==it->first){
          //This controller goes to this car.
          Vehicle_Entity* eVehicle = it->second.get();
          VehiclePtr pVeh = eVehicle->m_pVehicle;
          pVeh->setSteeringValue(pCarCon->m_dSteering, 0);
          pVeh->setSteeringValue(pCarCon->m_dSteering, 1);
          pVeh->applyEngineForce(pCarCon->m_dTorque, 2);
          pVeh->applyEngineForce(pCarCon->m_dTorque, 3);
        }
      }
    }

    ///////////////
    // GPS (TODO)
    ///////////////
    else if(Device->m_sDeviceType=="GPS"){
      SimGPS* pGPS = (SimGPS*) m_mDevices.at(ii);
      for(std::map<string, std::shared_ptr<Entity> > ::iterator it =
          m_mShapes.begin(); it!=m_mShapes.end(); it++ ){
        if(pGPS->GetBodyName()==it->first){
          //This controller goes to this car.
          Entity* eEntity = it->second.get();
          MotionStatePtr bodyMotion = eEntity->m_pMotionState;
          btTransform bodyTransform;
          btVector3 bodyPos = bodyTransform.getOrigin();
          bodyMotion->getWorldTransform(bodyTransform);
          Eigen::Vector3d bodyPose;
          bodyPose<<bodyPos.getX(), bodyPos.getY(), bodyPos.getZ();
          pGPS->Update(bodyPose);
        }
      }
    }

    ///////////////
    // IMU (TODO)
    ///////////////

  }
}

////////////////////////////////

void PhysicsEngine::StepSimulation(){
  m_pDynamicsWorld->stepSimulation( m_dTimeStep,  m_nMaxSubSteps );
  RunDevices();
  if(m_mRayVehicles.size()!=0){
    // Go through all of our vehicles and update their part poses.
    for(std::map<string, std::shared_ptr<Vehicle_Entity> > ::iterator it =
        m_mRayVehicles.begin(); it!=m_mRayVehicles.end(); it++ ){
      // The MotionStatePointer holds the ModelNode, which holds the poses.
      Vehicle_Entity* eVehicle = it->second.get();
      NodeMotionState* mMotion = eVehicle->m_pMotionState.get();
      SimRaycastVehicle* pVehicle = (SimRaycastVehicle*) &mMotion->object;
      std::vector<Eigen::Matrix4d> VehiclePoses =
          GetVehicleTransform(pVehicle->GetName());
      pVehicle->SetPose(SwitchYaw(_T2Cart(VehiclePoses.at(0))));
      pVehicle->SetWheelPose(1, SwitchWheelYaw(_T2Cart(VehiclePoses.at(1))));
      pVehicle->SetWheelPose(0, SwitchWheelYaw(_T2Cart(VehiclePoses.at(2))));
      pVehicle->SetWheelPose(2, SwitchWheelYaw(_T2Cart(VehiclePoses.at(3))));
      pVehicle->SetWheelPose(3, SwitchWheelYaw(_T2Cart(VehiclePoses.at(4))));
    }
  }
}

//////////////////////////////////////////////////////////
///
/// PRINT AND DRAW FUNCTIONS
///
//////////////////////////////////////////////////////////

void PhysicsEngine::PrintAllShapes(){
  for(std::map<string, std::shared_ptr<Entity> > ::iterator it =
      m_mShapes.begin(); it!=m_mShapes.end(); it++ ){
    std::cout<<it->first<<std::endl;
  }
}

//////////////////////////////////////////////////////////
///
/// GETTERS
///
//////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////
///
/// RAYCAST VEHICLE FUNCTIONS
///
//////////////////////////////////////////////////////////

/// These functions just help draw the car properly.

Eigen::Vector6d PhysicsEngine::SwitchYaw(Eigen::Vector6d bad_yaw){
  Eigen::Vector6d good_yaw;
  good_yaw<<bad_yaw(0), bad_yaw(1), bad_yaw(2),
      bad_yaw(4), bad_yaw(5), bad_yaw(3);
  return good_yaw;
}

Eigen::Vector6d PhysicsEngine::SwitchWheelYaw(Eigen::Vector6d bad_yaw){
  Eigen::Vector6d good_yaw;
  good_yaw<<bad_yaw(0), bad_yaw(1), bad_yaw(2),
      bad_yaw(4), -bad_yaw(3), bad_yaw(5);
  Eigen::Vector6d temp;
  temp<<0,0,0,M_PI/2,0,0;
  good_yaw = good_yaw+temp;
  return good_yaw;
}

///////////////////////////////////////////////////

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
  std::shared_ptr<Vehicle_Entity> std_Vehicle =
      m_mRayVehicles.at(sVehicleName);
  Vehicle_Entity* Vehicle = std_Vehicle.get();
  std::vector<Eigen::Matrix4d> Eig_transforms = GetVehiclePoses(Vehicle);
  return Eig_transforms;
}

/////////////////////////////////////////////////////
///// Since we're usually starting above the mesh, we use this function
///// to set the car on the ground before we start simulation.
/////

//double* RaycastToGround(double id, double x, double y){
//  double* pose = new double[3];
//  VehiclePtr Vehicle = m_mRayVehicles[id]->m_pVehicle;

//  // Move our vehicle out of the way...
//  btVector3 point(x+50, y+50, -100);
//  btMatrix3x3 rot = Vehicle->getChassisWorldTransform().getBasis();
//  btTransform bullet_trans(rot, point);
//  m_mRayVehicles[id]->m_pRigidBody->setCenterOfMassTransform(bullet_trans);

//  // Now shoot our ray...
//  btVector3 ray_start(x, y, 100);
//  btVector3 ray_end(x, y, -100);
//  btCollisionWorld::ClosestRayResultCallback ray_callback(ray_start,
//                                                          ray_end);
//  m_pDynamicsWorld->rayTest(ray_start, ray_end, ray_callback);
//  btVector3 hitpoint = Vehicle->getChassisWorldTransform().getOrigin();
//  if(ray_callback.hasHit()){
//    hitpoint = ray_callback.m_hitPointWorld;
//    btWheelInfo wheel = Vehicle->getWheelInfo(2);
//    double radius = wheel.m_wheelsRadius;
//    // Find a way to access the height of the car.
//    hitpoint.setZ(hitpoint.getZ()+(3*radius));
//  }

//  // Now move our car!
//  btTransform bullet_move(rot, hitpoint);
//  m_mRayVehicles[id]->m_pRigidBody->setCenterOfMassTransform(bullet_move);

//  //Now make sure none of our wheels are in the ground.
//  //Kind of a nasty oop, but keep it for now.
//  double hit = -1;
//  double count = 0;
//  while(hit==-1 && count<20){
//    for(int i = 0; i<4; i++){
//      hit = Vehicle->rayCast(Vehicle->getWheelInfo(i));
//      if(hit!=-1){
//        break;
//      }
//    }
//    //If we're still in the ground, lift us up!
//    hitpoint.setZ(hitpoint.getZ()+.1);
//    btTransform bullet_move(rot, hitpoint);
//    m_mRayVehicles[id]->
//        m_pRigidBody->setCenterOfMassTransform(bullet_move);
//    if(hit!=-1){
//      break;
//    }
//    count++;
//    if(count==20){
//     break;
//    }
//  }
//  int on = false;
//  btVector3 VehiclePose = Vehicle->getChassisWorldTransform().getOrigin();
//  while(on == 0){
//    on = OnTheGround(id);
//    StepSimulation();
//    VehiclePose = Vehicle->getChassisWorldTransform().getOrigin();
//  }

//  pose[0] = VehiclePose.getX();
//  pose[1] =VehiclePose.getY();
//  pose[2] = VehiclePose.getZ();
//  return pose;

//}

/////////////////////////////////////////////////////
//// This just drops us off on the surface...

//int OnTheGround(double id){
//  VehiclePtr Vehicle = m_mRayVehicles[id]->m_pVehicle;
//  int OnGround = 0;
//  int hit = 0;
//  for(int i = 0; i<4; i++){
//    hit = hit + Vehicle->rayCast(Vehicle->getWheelInfo(i));
//  }
//  if(hit==0){
//    OnGround = 1;
//  }
//  return OnGround;
//}

/////////////////////////////////////////////////////

//void SetVehicleVels(double id, double* lin_vel, double* ang_vel){
//  RigidBodyPtr VehicleBody = m_mRayVehicles[id]->m_pRigidBody;
//  VehiclePtr Vehicle = m_mRayVehicles[id]->m_pVehicle;
//  btVector3 Lin(lin_vel[0], lin_vel[1], lin_vel[2]);
//  btVector3 Ang(ang_vel[0], ang_vel[1], ang_vel[2]);
//  VehicleBody->setLinearVelocity(Lin);
//  VehicleBody->setAngularVelocity(Ang);
//  Vehicle->resetSuspension();
//}

/////////////////////////////////////////////////////
///// This function resets the car to its initial start pose for the mesh
///// we were just on (a set of poses we originally got from RaycastToGround.

//void ResetVehicle(double id, double* start_pose, double* start_rot){
//  // Move the vehicle into start position
//  btMatrix3x3 rot(start_rot[0], start_rot[3], start_rot[6],
//                  start_rot[1], start_rot[4], start_rot[7],
//                  start_rot[2], start_rot[5], start_rot[8]);
//  btVector3 pose(start_pose[0], start_pose[1], start_pose[2]);
//  btTransform bullet_trans(rot, pose);
//  // Reset our car to its initial state.
//  m_mRayVehicles[id]->m_pRigidBody->setCenterOfMassTransform(bullet_trans);
//}

/////////////////////////////////////////////////////
///// SPEED_COMMAND_SIM
///// When we just need state information from a RaycastVehicle, and rendering is
///// not an object, then we run the simulation as fast as possible. We can
///// just grab intermediate states, along with the end state, once the
///// simulation is finished.
////////

//double* SpeedSim(double id, double* start_pose, double* start_rot,
//                 double* start_lin_vel, double* start_ang_vel,
//                 double* forces, double* steering_angles,
//                 double command_length){
//  int state_size = (command_length*3)+22;
//  double* states = new double[state_size];
//  VehiclePtr Vehicle = m_mRayVehicles[id]->m_pVehicle;
//  ResetVehicle(id, start_pose, start_rot);
//  SetVehicleVels(id, start_lin_vel, start_ang_vel);

//  // Run our commands through
//  for(int i = 0; i < command_length; i++){
//    CommandRaycastVehicle(id, steering_angles[i], forces[i]);
//    StepSimulation();
//    btVector3 VehiclePose =
//        Vehicle->getChassisWorldTransform().getOrigin();
//    states[3*i] = VehiclePose.getX();
//    states[3*i+1] = VehiclePose.getY();
//    states[3*i+2] = VehiclePose.getZ();

//    // Get our whole state on the last step.

//    if(i==command_length-1){
//      btVector3 VehiclePose =
//          Vehicle->getChassisWorldTransform().getOrigin();
//      btMatrix3x3 VehicleRot =
//          Vehicle->getChassisWorldTransform().getBasis();
//      states[3*i+3] = VehiclePose.getX();
//      states[3*i+4] = VehiclePose.getY();
//      states[3*i+5] = VehiclePose.getZ();
//      states[3*i+6] = VehicleRot[0].getX();
//      states[3*i+7] = VehicleRot[1].getX();
//      states[3*i+8] = VehicleRot[2].getX();
//      states[3*i+9] = VehicleRot[0].getY();
//      states[3*i+10] = VehicleRot[1].getY();
//      states[3*i+11] = VehicleRot[2].getY();
//      states[3*i+12] = VehicleRot[0].getZ();
//      states[3*i+13] = VehicleRot[1].getZ();
//      states[3*i+14] = VehicleRot[2].getZ();
//      double* motionstate = GetRaycastMotionState( id );
//      states[3*i+15] = motionstate[2];
//      states[3*i+16] = motionstate[3];
//      states[3*i+17] = motionstate[4];
//      states[3*i+18] = motionstate[5];
//      states[3*i+19] = motionstate[6];
//      states[3*i+20] = motionstate[7];
//      states[3*i+21] = motionstate[8];
//    }
//  }

//  //Reset our vehicle again (just in case this is our last iteration)
//  ResetVehicle(id, start_pose, start_rot);
//  SetVehicleVels(id, start_lin_vel, start_ang_vel);
//  CommandRaycastVehicle(id, 0, 0);
//  return states;
//}

