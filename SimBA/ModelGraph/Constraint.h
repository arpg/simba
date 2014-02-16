/*

   Joint between links.

 */


#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

#include <ModelGraph/ModelNode.h>
#include <ModelGraph/Shape.h>

class Constraint : public ModelNode
{

  // I'm not sure what to put here; it's just useful for categorization.

};

//////////////////////////////////////////
/// TYPES OF CONSTRAINTS
/// The number after the typename in the class refers to the number of shapes
/// that the constraint connects; i.e. PToPOne is a constraint that
/// fixes the postion of one shape, while PToPTwo ties two shapes together.
//////////////////////////////////////////

//////////////
/// POINT-TO-POINT
//////////////

class PToPOne : public Constraint
{
public:
  PToPOne(std::string sName, Shape* Shape_A, Eigen::Vector3d pivot_in_A){
    SetName(sName);
    m_Shape_A = Shape_A->GetName();
    m_pivot_in_A = pivot_in_A;

    // Connect the shapes
    Shape_A->AddChild( this );
    this->m_pParent = Shape_A;
  }
  std::string m_Shape_A;
  Eigen::Vector3d m_pivot_in_A;
};

class PToPTwo : public Constraint
{
public:
  PToPTwo(std::string sName, Shape* Shape_A, Shape* Shape_B,
          Eigen::Vector3d pivot_in_A, Eigen::Vector3d pivot_in_B){
    SetName(sName);
    m_Shape_A = Shape_A->GetName();
    m_Shape_B = Shape_B->GetName();
    m_pivot_in_A = pivot_in_A;
    m_pivot_in_B = pivot_in_B;

    // Connect the shapes
    Shape_A->AddChild( this );
    this->AddChild( Shape_B );
    this->m_pParent = Shape_A;
    Shape_B->m_pParent = this;
  }
  std::string m_Shape_A;
  std::string m_Shape_B;
  Eigen::Vector3d m_pivot_in_A;
  Eigen::Vector3d m_pivot_in_B;
};

//////////////
/// HINGE
//////////////
class HingeOnePivot : public Constraint
{
public:
  HingeOnePivot(std::string sName, Shape* Shape_A,
                Eigen::Vector3d pivot_in_A,
                Eigen::Vector3d Axis_in_A){
    SetName(sName);
    m_Shape_A = Shape_A->GetName();
    m_axis_in_A = Axis_in_A;
    m_pivot_in_A = pivot_in_A;
    m_low_limit = -1;
    m_high_limit = 1;
    m_softness = .9;
    m_bias =  .3;
    m_relaxation = 1;

    // Connect the shapes
    Shape_A->AddChild( this );
    this->m_pParent = Shape_A;
  }

  void SetLimits(  double low_limit, double high_limit, double softness,
                   double bias, double relaxation){
    m_low_limit = low_limit;
    m_high_limit = high_limit;
    m_softness = softness;
    m_bias =  bias;
    m_relaxation = relaxation;
  }

  std::string m_Shape_A;
  Eigen::Vector3d m_axis_in_A;
  Eigen::Vector3d m_pivot_in_A;
  double m_low_limit;
  double m_high_limit;
  double m_softness;
  double m_bias;
  double m_relaxation;

};

class HingeTwoPivot : public Constraint
{
public:
  HingeTwoPivot(std::string sName, Shape* Shape_A, Shape* Shape_B,
                Eigen::Vector3d pivot_in_A, Eigen::Vector3d pivot_in_B,
                Eigen::Vector3d Axis_in_A, Eigen::Vector3d Axis_in_B){
    SetName(sName);
    m_Shape_A = Shape_A->GetName();
    m_Shape_B = Shape_B->GetName();
    m_axis_in_A = Axis_in_A;
    m_axis_in_B = Axis_in_B;
    m_pivot_in_A = pivot_in_A;
    m_pivot_in_B = pivot_in_B;
    m_low_limit = -1;
    m_high_limit = 1;
    m_softness = .9;
    m_bias =  .3;
    m_relaxation = 1;

    // Connect the shapes
    Shape_A->AddChild( this );
    this->AddChild( Shape_B );
    this->m_pParent = Shape_A;
    Shape_B->m_pParent = this;
  }

  void SetLimits(  double low_limit, double high_limit, double softness,
                   double bias, double relaxation){
    m_low_limit = low_limit;
    m_high_limit = high_limit;
    m_softness = softness;
    m_bias =  bias;
    m_relaxation = relaxation;
  }

  std::string m_Shape_A;
  std::string m_Shape_B;
  Eigen::Vector3d m_axis_in_A;
  Eigen::Vector3d m_axis_in_B;
  Eigen::Vector3d m_pivot_in_A;
  Eigen::Vector3d m_pivot_in_B;
  double m_low_limit;
  double m_high_limit;
  double m_softness;
  double m_bias;
  double m_relaxation;

};

////////
/// HINGE 2
////////

class Hinge2 : public Constraint{
public:
  Hinge2(std::string sName, Shape* Shape_A, Shape* Shape_B,
         Eigen::Vector3d Anchor,
         Eigen::Vector3d Axis_1, Eigen::Vector3d Axis_2){
    SetName(sName);
    m_Shape_A = Shape_A->GetName();
    m_Shape_B = Shape_B->GetName();
    m_Anchor = Anchor;
    m_Axis_1 = Axis_1;
    m_Axis_2 = Axis_2;
    m_damping = 50;
    m_stiffness = 1;

    // Connect the shapes
    Shape_A->AddChild( this );
    this->AddChild( Shape_B );
    this->m_pParent = Shape_A;
    Shape_B->m_pParent = this;
  }


  void SetLimits(  double damping, double stiffness,
                   Eigen::Vector3d LinLowLimit, Eigen::Vector3d LinUppLimit,
                   Eigen::Vector3d AngLowLimit, Eigen::Vector3d AngUppLimit){
    m_damping = damping;
    m_stiffness = stiffness;
    m_LowerLinLimit = LinLowLimit;
    m_UpperLinLimit = LinUppLimit;
    m_LowerAngLimit = AngLowLimit;
    m_UpperAngLimit = AngUppLimit;
  }

  std::string m_Shape_A;
  std::string m_Shape_B;
  Eigen::Vector3d m_Anchor;
  Eigen::Vector3d m_Axis_1;
  Eigen::Vector3d m_Axis_2;
  double m_damping;
  double m_stiffness;
  Eigen::Vector3d m_LowerLinLimit;
  Eigen::Vector3d m_UpperLinLimit;
  Eigen::Vector3d m_LowerAngLimit;
  Eigen::Vector3d m_UpperAngLimit;


};


//////////
///// Six DOF
//////////

// TODO: Implement these.


//int SixDOF_one(double id_A, double* transform_A, double* limits){
//  boost::shared_ptr<Shape_Entity> Shape_A = m_mShapes.at(id_A);
//  btQuaternion quat_A(transform_A[3], transform_A[4],
//      transform_A[5], transform_A[6]);
//  btVector3 pos_A(transform_A[0], transform_A[1], transform_A[2]);
//  btTransform trans_A(quat_A, pos_A);
//  btGeneric6DofConstraint* SixDOF =
//      new btGeneric6DofConstraint(*Shape_A->m_pRigidBody.get(),
//                                  trans_A,
//                                  true);
//  btVector3 max_lin_limits(limits[0], limits[1],  limits[2]);
//  btVector3 min_lin_limits(limits[3], limits[4],  limits[5]);
//  btVector3 max_ang_limits(limits[6], limits[7],  limits[8]);
//  btVector3 min_ang_limits(limits[9], limits[10], limits[11]);
//  SixDOF->setLinearLowerLimit(min_lin_limits);
//  SixDOF->setLinearUpperLimit(max_lin_limits);
//  SixDOF->setAngularLowerLimit(min_ang_limits);
//  SixDOF->setAngularUpperLimit(max_ang_limits);
//  m_pDynamicsWorld->addConstraint(SixDOF);
//  int id = m_SixDOF.size();
//  m_SixDOF.push_back(SixDOF);
//  return id;
//}

//int SixDOF_two(double id_A, double id_B,
//                double* transform_A, double* transform_B,
//                double* limits){
//  boost::shared_ptr<Shape_Entity> Shape_A = m_mShapes.at(id_A);
//  boost::shared_ptr<Shape_Entity> Shape_B = m_mShapes.at(id_B);
//  btQuaternion quat_A(transform_A[3], transform_A[4],
//      transform_A[5], transform_A[6]);
//  btVector3 pos_A(transform_A[0], transform_A[1], transform_A[2]);
//  btTransform trans_A(quat_A, pos_A);
//  btQuaternion quat_B(transform_B[3], transform_B[4],
//      transform_B[5], transform_B[6]);
//  btVector3 pos_B(transform_B[0], transform_B[1], transform_B[2]);
//  btTransform trans_B(quat_B, pos_B);
//  btGeneric6DofConstraint* SixDOF =
//      new btGeneric6DofConstraint(*Shape_A->m_pRigidBody.get(),
//                                  *Shape_B->m_pRigidBody.get(),
//                                  trans_A, trans_B,
//                                  true);
//  btVector3 max_lin_limits(limits[0], limits[1],  limits[2]);
//  btVector3 min_lin_limits(limits[3], limits[4],  limits[5]);
//  btVector3 max_ang_limits(limits[6], limits[7],  limits[8]);
//  btVector3 min_ang_limits(limits[9], limits[10], limits[11]);
//  SixDOF->setLinearLowerLimit(min_lin_limits);
//  SixDOF->setLinearUpperLimit(max_lin_limits);
//  SixDOF->setAngularLowerLimit(min_ang_limits);
//  SixDOF->setAngularUpperLimit(max_ang_limits);
//  m_pDynamicsWorld->addConstraint(SixDOF);
//  int id = m_SixDOF.size();
//  m_SixDOF.push_back(SixDOF);
//  return id;
//}


#endif // CONSTRAINT_H_

