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

  // TODO: Complete this function for all available Devices
  // This function takes all of our controller/sensors and processes
  // whatever they need to do. For instance, the CarController would
  // command the RaycastVehicle in a certain way, and all of the GPS
  // and IMU would register the states of the physics.

}

////////////////////////////////

void PhysicsEngine::StepSimulation(){
  m_pDynamicsWorld->stepSimulation( m_dTimeStep,  m_nMaxSubSteps );
  RunDevices();
  if(m_mRayVehicles.size()!=0){
    // Go through all of our vehicles and update their part poses.
    for(std::map<string, boost::shared_ptr<Vehicle_Entity> > ::iterator it =
        m_mRayVehicles.begin(); it!=m_mRayVehicles.end(); it++ ){
      // The MotionStatePointer holds the ModelNode, which holds the poses.
      Vehicle_Entity* eVehicle = it->second.get();
      NodeMotionState* mMotion = eVehicle->m_pMotionState.get();

      // TODO: make the physics react to all of the commands coming
      // through Node.

      //HOW TO PASS COMMANDS TO THE CAR:
//      VehiclePtr pVeh = eVehicle->m_pVehicle;
//      pVeh->setSteeringValue(M_PI/6, 0);
//      pVeh->setSteeringValue(M_PI/6, 1);
//      pVeh->applyEngineForce(10, 2);
//      pVeh->applyEngineForce(10, 3);

      RaycastVehicle* pVehicle = (RaycastVehicle*) &mMotion->object;
      std::vector<Eigen::Matrix4d> VehiclePoses =
          GetVehicleTransform(pVehicle->GetName());
      pVehicle->SetPose(SwitchYaw(_T2Cart(VehiclePoses.at(0))));
      pVehicle->SetWheelPose(0, SwitchWheelYaw(_T2Cart(VehiclePoses.at(1))));
      pVehicle->SetWheelPose(1, SwitchWheelYaw(_T2Cart(VehiclePoses.at(2))));
      pVehicle->SetWheelPose(2, SwitchWheelYaw(_T2Cart(VehiclePoses.at(3))));
      pVehicle->SetWheelPose(3, SwitchWheelYaw(_T2Cart(VehiclePoses.at(4))));

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
/// VEHICLE POSE GETTERS
///
//////////////////////////////////////////////////////////

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
