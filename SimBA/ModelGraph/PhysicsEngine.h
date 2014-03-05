#ifndef _PHYSICS_ENGINE_H
#define _PHYSICS_ENGINE_H

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
/// PhysicsEngine encapsulates all of the Physics engine (Bullet) into one
/// class. It initializes the physics environment, and allows for the addition
/// and deletion of objects. It must also be called to run the physics sim.
///
//////////////////////////////////////////////////////////


class PhysicsEngine
{
public:

  /// CONSTRUCTOR
  PhysicsEngine();

  /// Initializer
  bool Init(double dGravity = -9.8, double dTimeStep = 1.0/30.0,
            double nMaxSubSteps = 10);

  /// ADDING OBJECTS TO THE PHYSICS ENGINE
  /// RegisterObject adds shapes, constraints, and vehicles.
  void RegisterObject(ModelNode *pItem);
  void RegisterDevice(SimDeviceInfo* pDevice);
  bool isVehicle(string Shape);

  /// RUNNING THE SIMULATION
  void DebugDrawWorld();
  void RunDevices();
  void StepSimulation();

  /// PRINT FUNCTIONS
  void PrintAllShapes();

  /// GETTERS - Used for sensors and controllers
  btHinge2Constraint* getHinge2Constraint(string name);
  btHingeConstraint* getHingeConstraint(string name);
  /// RAYCAST VEHICLE METHODS
  Eigen::Vector6d SwitchYaw(Eigen::Vector6d bad_yaw);
  Eigen::Vector6d SwitchWheelYaw(Eigen::Vector6d bad_yaw);
  std::vector<Eigen::Matrix4d> GetVehiclePoses( Vehicle_Entity* Vehicle );
  std::vector<Eigen::Matrix4d> GetVehicleTransform(std::string sVehicleName);

  /// PUBLIC MEMBER VARIABLES
  DebugDraw                                               m_DebugDrawer;
  std::map<string, boost::shared_ptr<Vehicle_Entity> >    m_mRayVehicles;
  std::map<string, boost::shared_ptr<Entity> >            m_mShapes;
  std::map<string, boost::shared_ptr<Compound_Entity> >   m_mCompounds;
  std::map<string, btHingeConstraint*>                    m_mHinge;
  std::map<string, btHinge2Constraint*>                   m_mHinge2;
  std::map<string, btGeneric6DofConstraint*>              m_mSixDOF;
  std::map<string, btPoint2PointConstraint*>              m_mPtoP;
  vector<SimDeviceInfo*>                                  m_mDevices;
  boost::shared_ptr<btDiscreteDynamicsWorld>              m_pDynamicsWorld;

private:

  /// PRIVATE MEMBER VARIABLES
  btDefaultCollisionConfiguration                        m_CollisionConfiguration;
  boost::shared_ptr<btCollisionDispatcher>               m_pDispatcher;
  boost::shared_ptr<btDbvtBroadphase>                    m_pBroadphase;
  boost::shared_ptr<btSequentialImpulseConstraintSolver> m_pSolver;
  double                                                 m_dTimeStep;
  double                                                 m_dGravity;
  int                                                    m_nMaxSubSteps;

};

#endif //PHYSICSENGINE_H
