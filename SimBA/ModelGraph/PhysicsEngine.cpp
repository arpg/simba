#include "PhysicsEngine.h"
#include <thread>

//////////////////////////////////////////////////////////
///
/// CONSTRUCTOR
///
//////////////////////////////////////////////////////////

PhysicsEngine::PhysicsEngine(){
  timestep_ = 1.0/30.0;
  gravity_acc_ = -9.8;
  time_max_substeps_ = 10; // bullet -- for stepSimulation
}

bool PhysicsEngine::Init(double dGravity, double dTimeStep,
                         double nMaxSubSteps){
  timestep_    = dTimeStep;
  gravity_acc_     = dGravity;
  time_max_substeps_ = nMaxSubSteps;

  // Physics stuff
  // See http://bulletphysics.org/mediawiki-1.5.8/index.php/Hello_World

  std::shared_ptr<btCollisionDispatcher> Dispatcher(
      new btCollisionDispatcher(&collision_configuration_) );
  bt_dispatcher_ = Dispatcher;

  std::shared_ptr<btDbvtBroadphase> Broadphase( new btDbvtBroadphase );
  bt_broadphase_ = Broadphase;

  std::shared_ptr<btSequentialImpulseConstraintSolver> Solver(
      new btSequentialImpulseConstraintSolver );
  bt_solver_ = Solver;
  std::shared_ptr<btDiscreteDynamicsWorld> DWorld(
      new btDiscreteDynamicsWorld(bt_dispatcher_.get(),
                                  bt_broadphase_.get(),
                                  bt_solver_.get(),
                                  &collision_configuration_) );
  dynamics_world_ = DWorld;
  dynamics_world_->setGravity( btVector3(0, 0,  gravity_acc_) );
  dynamics_world_->setDebugDrawer( &debug_drawer_ );
  dynamics_world_->getDebugDrawer()->
      setDebugMode(btIDebugDraw::DBG_DrawWireframe +
                   btIDebugDraw::DBG_FastWireframe +
                   btIDebugDraw::DBG_DrawConstraints);
  vehicle_raycaster_ = new btDefaultVehicleRaycaster(dynamics_world_.get());
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
      bullet_vehicle btRayVehicle( pItem, dynamics_world_.get(),
                                   vehicle_raycaster_);
      CollisionShapePtr pShape( btRayVehicle.getBulletShapePtr() );
      MotionStatePtr pMotionState( btRayVehicle.getBulletMotionStatePtr() );
      RigidBodyPtr body( btRayVehicle.getBulletBodyPtr() );
      VehiclePtr vehicle( btRayVehicle.getBulletRaycastVehicle() );
      std::shared_ptr<Vehicle_Entity> pEntity( new Vehicle_Entity );
      pEntity->m_pRigidBody = body;
      pEntity->m_pShape = pShape;
      pEntity->m_pMotionState = pMotionState;
      pEntity->m_pVehicle = vehicle;
      ray_vehicles_map_[pItem->GetName()] = pEntity;
    }

    //Box
    if (dynamic_cast<BoxShape*>( pNodeShape ) != NULL) {
      bullet_cube btBox(pItem);
      CollisionShapePtr pShape( btBox.getBulletShapePtr() );
      MotionStatePtr pMotionState( btBox.getBulletMotionStatePtr() );
      RigidBodyPtr body( btBox.getBulletBodyPtr() );
      dynamics_world_->addRigidBody( body.get() );

      //Save the object; easier deconstruction this way.
      std::shared_ptr<Entity> pEntity( new Entity );
      pEntity->m_pRigidBody = body;
      pEntity->m_pShape = pShape;
      pEntity->m_pMotionState = pMotionState;
      shapes_map_[pItem->GetName()] = pEntity;
    }

    //Cylinder
    else if (dynamic_cast<CylinderShape*>( pNodeShape ) != NULL) {
      bullet_cylinder btCylinder(pItem);
      CollisionShapePtr pShape( btCylinder.getBulletShapePtr() );
      MotionStatePtr pMotionState( btCylinder.getBulletMotionStatePtr() );
      RigidBodyPtr body( btCylinder.getBulletBodyPtr() );
      dynamics_world_->addRigidBody( body.get() );

      //Save the object; easier deconstruction this way.
      std::shared_ptr<Entity> pEntity( new Entity );
      pEntity->m_pRigidBody = body;
      pEntity->m_pShape = pShape;
      pEntity->m_pMotionState = pMotionState;
      shapes_map_[pItem->GetName()] = pEntity;
    }

    //Plane
    else if (dynamic_cast<PlaneShape*>( pNodeShape ) != NULL) {
      bullet_plane btPlane(pItem);
      CollisionShapePtr pShape( btPlane.getBulletShapePtr() );
      MotionStatePtr pMotionState( btPlane.getBulletMotionStatePtr() );
      RigidBodyPtr body( btPlane.getBulletBodyPtr() );
      dynamics_world_->addRigidBody( body.get() );

      //Save the object; easier deconstruction this way.
      std::shared_ptr<Entity> pEntity( new Entity );
      pEntity->m_pRigidBody = body;
      pEntity->m_pShape = pShape;
      pEntity->m_pMotionState = pMotionState;
      shapes_map_[pItem->GetName()] = pEntity;
    }

    //Heightmap
    else if (dynamic_cast<HeightmapShape*>( pNodeShape ) != NULL) {
      bullet_heightmap btMap(pItem);
      CollisionShapePtr pShape( btMap.getBulletShapePtr() );
      MotionStatePtr pMotionState( btMap.getBulletMotionStatePtr() );
      RigidBodyPtr body( btMap.getBulletBodyPtr() );
      dynamics_world_->addRigidBody( body.get() );

      //Save the object; easier deconstruction this way.
      std::shared_ptr<Entity> pEntity( new Entity );
      pEntity->m_pRigidBody = body;
      pEntity->m_pShape = pShape;
      pEntity->m_pMotionState = pMotionState;
      shapes_map_[pItem->GetName()] = pEntity;
    }

    //Mesh
    else if (dynamic_cast<MeshShape*>( pNodeShape ) != NULL){
      bullet_mesh btMesh(pItem);
      CollisionShapePtr pShape( btMesh.getBulletShapePtr() );
      MotionStatePtr pMotionState( btMesh.getBulletMotionStatePtr() );
      RigidBodyPtr body( btMesh.getBulletBodyPtr() );
      dynamics_world_->addRigidBody( body.get() );

      //Save the object; easier deconstruction this way.
      std::shared_ptr<Entity> pEntity( new Entity );
      pEntity->m_pRigidBody = body;
      pEntity->m_pShape = pShape;
      pEntity->m_pMotionState = pMotionState;
      shapes_map_[pItem->GetName()] = pEntity;
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
        std::shared_ptr<Vehicle_Entity> Shape_A = ray_vehicles_map_.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_A = shapes_map_.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      btVector3 pivot_A(pCon->m_pivot_in_A[0], pCon->m_pivot_in_A[1],
                        pCon->m_pivot_in_A[2]);
      btPoint2PointConstraint* PToP =
          new btPoint2PointConstraint(*RigidShape_A, pivot_A);
      dynamics_world_->addConstraint(PToP);
      ptop_map_[pCon->GetName()] = PToP;
    }

    else if(dynamic_cast<PToPTwo*>( pNodeCon ) != NULL) {
      PToPTwo* pCon = (PToPTwo*) pNodeCon;
      btRigidBody* RigidShape_A;
      btRigidBody* RigidShape_B;
      if(isVehicle(pCon->m_Shape_A)){
        std::shared_ptr<Vehicle_Entity> Shape_A = ray_vehicles_map_.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_A = shapes_map_.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      if(isVehicle(pCon->m_Shape_B)){
        std::shared_ptr<Vehicle_Entity> Shape_B = ray_vehicles_map_.at(pCon->m_Shape_B);
        RigidShape_B = Shape_B->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_B = shapes_map_.at(pCon->m_Shape_B);
        RigidShape_B = Shape_B->m_pRigidBody.get();
      }
      btVector3 pivot_A(pCon->m_pivot_in_A[0], pCon->m_pivot_in_A[1],
                        pCon->m_pivot_in_A[2]);
      btVector3 pivot_B(pCon->m_pivot_in_B[0], pCon->m_pivot_in_B[1],
                        pCon->m_pivot_in_B[2]);
      btPoint2PointConstraint* PToP =
          new btPoint2PointConstraint(*RigidShape_A, *RigidShape_B,
                                      pivot_A, pivot_B);
      dynamics_world_->addConstraint(PToP);
      ptop_map_[pCon->GetName()] = PToP;
    }

    //Hinge
    else if(dynamic_cast<HingeOnePivot*>( pNodeCon ) != NULL) {
      HingeOnePivot* pCon = (HingeOnePivot*) pNodeCon;
      btRigidBody* RigidShape_A;
      if(isVehicle(pCon->m_Shape_A)){
        std::shared_ptr<Vehicle_Entity> Shape_A = ray_vehicles_map_.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_A = shapes_map_.at(pCon->m_Shape_A);
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
      dynamics_world_->addConstraint(Hinge);
      hinge_map_[pCon->GetName()] = Hinge;
    }

    else if(dynamic_cast<HingeTwoPivot*>( pNodeCon ) != NULL) {
      HingeTwoPivot* pCon = (HingeTwoPivot*) pNodeCon;
      btRigidBody* RigidShape_A;
      btRigidBody* RigidShape_B;
      if(isVehicle(pCon->m_Shape_A)){
        std::shared_ptr<Vehicle_Entity> Shape_A = ray_vehicles_map_.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_A = shapes_map_.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      if(isVehicle(pCon->m_Shape_B)){
        std::shared_ptr<Vehicle_Entity> Shape_B = ray_vehicles_map_.at(pCon->m_Shape_B);
        RigidShape_B = Shape_B->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_B = shapes_map_.at(pCon->m_Shape_B);
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
      dynamics_world_->addConstraint(Hinge);
      hinge_map_[pCon->GetName()] = Hinge;
    }

    //Hinge2
    else if(dynamic_cast<Hinge2*>( pNodeCon ) != NULL) {
      Hinge2* pCon = (Hinge2*) pNodeCon;
      btRigidBody* RigidShape_A;
      btRigidBody* RigidShape_B;
      if(isVehicle(pCon->m_Shape_A)){
        std::shared_ptr<Vehicle_Entity> Shape_A = ray_vehicles_map_.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_A = shapes_map_.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      if(isVehicle(pCon->m_Shape_B)){
        std::shared_ptr<Vehicle_Entity> Shape_B = ray_vehicles_map_.at(pCon->m_Shape_B);
        RigidShape_B = Shape_B->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_B = shapes_map_.at(pCon->m_Shape_B);
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
      dynamics_world_->addConstraint(Hinge);
      hinge2_map_[pCon->GetName()] = Hinge;
    }

    //SixDOF
    else if(dynamic_cast<SixDOFOne*>( pNodeCon ) != NULL) {
      SixDOFOne* pCon = (SixDOFOne*) pNodeCon;
      std::shared_ptr<Entity> Shape_A = shapes_map_.at(pCon->m_Shape_A);
      btTransform trans_A = toBullet(_Cart2T(pCon->m_Transform_A));
      btGeneric6DofConstraint* SixDOF =
          new btGeneric6DofConstraint(*Shape_A->m_pRigidBody.get(),
                                      trans_A,
                                      true);
      SixDOF->setLinearLowerLimit(toBulletVec3(pCon->m_LowerLinLimit));
      SixDOF->setLinearUpperLimit(toBulletVec3(pCon->m_UpperLinLimit));
      SixDOF->setAngularLowerLimit(toBulletVec3(pCon->m_LowerAngLimit));
      SixDOF->setAngularUpperLimit(toBulletVec3(pCon->m_UpperAngLimit));
      dynamics_world_->addConstraint(SixDOF);
      sixdof_map_[pCon->GetName()] = SixDOF;
    }

    else if(dynamic_cast<SixDOFTwo*>( pNodeCon ) != NULL) {
      SixDOFTwo* pCon = (SixDOFTwo*) pNodeCon;
      btRigidBody* RigidShape_A;
      btRigidBody* RigidShape_B;
      if(isVehicle(pCon->m_Shape_A)){
        std::shared_ptr<Vehicle_Entity> Shape_A = ray_vehicles_map_.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_A = shapes_map_.at(pCon->m_Shape_A);
        RigidShape_A = Shape_A->m_pRigidBody.get();
      }
      if(isVehicle(pCon->m_Shape_B)){
        std::shared_ptr<Vehicle_Entity> Shape_B = ray_vehicles_map_.at(pCon->m_Shape_B);
        RigidShape_B = Shape_B->m_pRigidBody.get();
      }
      else{
        std::shared_ptr<Entity> Shape_B = shapes_map_.at(pCon->m_Shape_B);
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
      dynamics_world_->addConstraint(SixDOF);
      sixdof_map_[pCon->GetName()] = SixDOF;
    }
  }

  return;

}

///////////////////////////////////////////////////////

void PhysicsEngine::RegisterDevice(SimDeviceInfo* pDevice){
  sim_devices_.push_back(pDevice);
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
  dynamics_world_->debugDrawWorld();
}


void PhysicsEngine::RunDevices(){
  for(unsigned int ii=0; ii<sim_devices_.size(); ii++){
    ///////////////
    // CAR CONTROLLER
    ///////////////
    SimDeviceInfo* Device = sim_devices_.at(ii);
    if(Device->m_sDeviceType=="CarController"){
      // We have to check all controllers.
      CarController* pCarCon = (CarController*) sim_devices_.at(ii);
      for(std::map<string, std::shared_ptr<Vehicle_Entity> > ::iterator it =
              ray_vehicles_map_.begin(); it!=ray_vehicles_map_.end(); it++ ){
        if(pCarCon->GetBodyName()==it->first){
          //This controller goes to this car.
          Vehicle_Entity* eVehicle = it->second.get();
          VehiclePtr pVeh = eVehicle->m_pVehicle;
          SimRaycastVehicle* vehicle_shape =
              (SimRaycastVehicle*)&eVehicle->m_pMotionState.get()->object;
          btVector3 vel = pVeh->getRigidBody()->getLinearVelocity();
          Eigen::Vector3d lin_vel;
          lin_vel<<vel.getX(), vel.getY(), vel.getZ();
          vehicle_shape->CommandCar(pCarCon->m_dSteering,
                                    pCarCon->m_dTorque,
                                    pCarCon->delta_time,
                                    lin_vel);
          pVeh->setSteeringValue(vehicle_shape->wheel_angles.at(0), 0);
          pVeh->setSteeringValue(vehicle_shape->wheel_angles.at(1), 1);
          pVeh->applyEngineForce(vehicle_shape->driving_force, 2);
          pVeh->applyEngineForce(vehicle_shape->driving_force, 3);
          // OPTION: Update the world only if we have commands.
          // dynamics_world_->stepSimulation( timestep_,  time_max_substeps_ );
        }
      }
    }

    ///////////////
    // GPS (TODO)
    ///////////////
    else if(Device->m_sDeviceType=="GPS"){
      SimGPS* pGPS = (SimGPS*) sim_devices_.at(ii);
      for(std::map<string, std::shared_ptr<Entity> > ::iterator it =
              shapes_map_.begin(); it!=shapes_map_.end(); it++ ){
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
  dynamics_world_->stepSimulation( timestep_,  time_max_substeps_ );
  // std::this_thread::sleep_for(std::chrono::milliseconds(50));
  RunDevices();
  if(ray_vehicles_map_.size()!=0){
    // Go through all of our vehicles and update their part poses.
    for(std::map<string, std::shared_ptr<Vehicle_Entity> > ::iterator it =
            ray_vehicles_map_.begin(); it!=ray_vehicles_map_.end(); it++ ){
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
          shapes_map_.begin(); it!=shapes_map_.end(); it++ ){
    std::cout<<it->first<<std::endl;
  }
}

//////////////////////////////////////////////////////////
///
/// GETTERS
///
//////////////////////////////////////////////////////////

btHinge2Constraint* PhysicsEngine::getHinge2Constraint(string name){
  std::map<string, btHinge2Constraint*>::iterator iter = hinge2_map_.find(name);
  if(iter!=hinge2_map_.end()){
    btHinge2Constraint* pHJ2 = hinge2_map_.find(name)->second;
    return pHJ2;
  }
  else{
    cout<<"Fatal Error! Cannot get Hinge2joint "<<name<<endl;
    btHinge2Constraint* pHJ2 = hinge2_map_.find(name)->second;
    return pHJ2;
  }
}

///////////////////////////////////////////////////////////////////

btHingeConstraint* PhysicsEngine::getHingeConstraint(string name){
  std::map<string, btHingeConstraint*>::iterator iter = hinge_map_.find(name);
  if(iter!=hinge_map_.end()){
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
  // Eigen::Vector6d good_yaw;
  // good_yaw<<bad_yaw(0), bad_yaw(1), bad_yaw(2),
  //     bad_yaw(3), -bad_yaw(4), bad_yaw(5);
  // Eigen::Vector6d temp;
  // // temp<<0,0,0,0,0,0;
  // temp<<0,0,0,M_PI/2,0,0;
  // good_yaw = good_yaw+temp;
  // return good_yaw;
  Eigen::Vector6d good_yaw;
  good_yaw<<bad_yaw(0), bad_yaw(1), bad_yaw(2),
      bad_yaw(4), -bad_yaw(3), bad_yaw(5);
  Eigen::Vector6d temp;
  temp<<0,0,0,M_PI/2,0,0;
  good_yaw = good_yaw+temp;
  return good_yaw;
}

///////////////////////////////////////////////////

std::vector< Eigen::Matrix4d > PhysicsEngine::GetVehiclePoses(
    Vehicle_Entity* Vehicle ){
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

std::vector<Eigen::Matrix4d> PhysicsEngine::GetVehicleTransform(
    std::string sVehicleName){
  std::shared_ptr<Vehicle_Entity> std_Vehicle =
      ray_vehicles_map_.at(sVehicleName);
  Vehicle_Entity* Vehicle = std_Vehicle.get();
  std::vector<Eigen::Matrix4d> Eig_transforms = GetVehiclePoses(Vehicle);
  return Eig_transforms;
}

////////////////////////////////////////////////////////////////////////////////
bool PhysicsEngine::RayCast(const Eigen::Vector3d& dSource,
                            const Eigen::Vector3d& dRayVector,
                            Eigen::Vector3d& dIntersect,
                            const bool& biDirectional){
  btVector3 source(dSource[0],dSource[1],dSource[2]);
  btVector3 vec(dRayVector[0],dRayVector[1],dRayVector[2]);
  btVector3 target = source + vec;
  btVehicleRaycaster::btVehicleRaycasterResult results;
  if( biDirectional ){
    source = source - vec;
  }
  if(vehicle_raycaster_->castRay(source,target,results) == 0){
    return false;
  } else {
    Eigen::Vector3d dNewSource(source[0],source[1],source[2]);
    dIntersect = dNewSource + results.m_distFraction *
        (biDirectional ? (Eigen::Vector3d)(dRayVector*2) : dRayVector);
    return true;
  }
}

////////////////////////////////////////////////////////////////////////////////
bool PhysicsEngine::RayCastNormal(const Eigen::Vector3d& dSource,
                                  const Eigen::Vector3d& dRayVector,
                                  Eigen::Vector3d& dNormal){
  bool hit = false;
  btVector3 source(dSource[0],dSource[1],dSource[2]);
  btVector3 vec(dRayVector[0],dRayVector[1],dRayVector[2]);
  source = source - vec*10;
  btVector3 target = source + vec*20;
  btCollisionWorld::ClosestRayResultCallback rayCallback(source, target);
  dynamics_world_->rayTest(source, target, rayCallback);
  //We need a default rotation... just make it the gravity vector, why not.
  btVector3 dNewNorm = dynamics_world_->getGravity();
  Eigen::Vector3d norm(dNewNorm[0], dNewNorm[1], dNewNorm[2]);
  dNormal = norm;
  if (rayCallback.hasHit()){
    dNewNorm = rayCallback.m_hitNormalWorld;
    Eigen::Vector3d norm(dNewNorm[0], dNewNorm[1], dNewNorm[2]);
    dNormal = norm;
    hit = true;
  }
  return hit;
}
