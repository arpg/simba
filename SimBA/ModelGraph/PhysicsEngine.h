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
  bool isVehicle(string Shape);

  /// RUNNING THE SIMULATION
  void DebugDrawWorld();
  void StepSimulation();

  /// PRINT FUNCTIONS
  void PrintAllShapes();

  /// OBJECT DELETION
  void DeleteHingeConstraintFromDynamicsWorld(string sConstraintName);
  void DeleteHinge2ConstraintFromDynamicsWorld(string sConstraintName);
  void DeleteRigidBodyFromDynamicsWorld(string sShapeFullName);
  void EraseEntityInShapeList(string sEntityName);

  /// GETTERS, SETTERS, AND FORCE FUNCTIONS
  /// TODO: Most of these have been grandfathered in, so there will have to be
  /// some pruning in the future.
  Entity getEntity(string name);
  vector<string> GetAllEntityName();
  btHinge2Constraint* getHinge2Constraint(string name);
  btHingeConstraint* getHingeConstraint(string name);
  btDynamicsWorld* GetDynamicsWorld();
  Eigen::Vector6d GetEntity6Pose( string name );
  void GetEntity6Pose(string name, Eigen::Vector6d& rPose);
  void SetEntity6Pose(string sName, Eigen::Vector6d Pose);
  Eigen::Vector3d GetEntityOrigin(string sName);
  Eigen::Matrix3d GetEntityBasis(string sName);
  void SetEntityOrigin(string sName, Eigen::Vector3d eOrigin);
  void SetEntityBasis(string sName, Eigen::Matrix3d mBasis);

  Eigen::Vector3d GetEntityLinearVelocity(string sBodyFullName);
  double GetEntityVelocity(string name);
  Eigen::Vector3d GetEntityAngularVelocity(string sBodyFullName);
  void SetEntityLinearvelocity(string sBodyFullName,
                               Eigen::Vector3d eLinearVelocity);
  void SetEntityAngularvelocity(string sBodyFullName,
                                Eigen::Vector3d eAngularVelocity);
  void SetEntityRotation(string EntityName, double roll, double pitch,
                         double yaw);
  void GetEntityRotation(string EntityName, double& roll, double& pitch,
                         double& yaw);
  void PrintEntityRotation(string EntityName);
  void SetFriction(string name, double F);
  void ApplyForceToEntity(string name, double F);
  void ApplyTorque(string sBodyFullName, Eigen::Vector3d eTorque);
  void ApplySteering(string sBodyFullName, Eigen::Vector3d eSteering);
  void SerializeRigidBodyToChar(string sBodyName, const unsigned char*& pData,
                                int& iDataSize);

  /// RAYCAST VEHICLE METHODS
  std::vector< Eigen::Matrix4d > GetVehiclePoses( Vehicle_Entity* Vehicle );
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
