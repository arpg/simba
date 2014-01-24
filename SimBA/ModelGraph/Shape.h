#ifndef SHAPE_H
#define SHAPE_H

#include <ModelGraph/ModelNode.h>

/////////////////////////////////////////
/// The Shape class
/// A shell for the ModelGraph-Shape subtype
/// It is a superclass holding all of the essential functions and members of our
/// Physics/ModelGraph Shapes.
/////////////////////////////////////////

//// Bullet instantiations reside in PhysicsEngine
//// SceneGraph instantiations are in RenderEngine (rename RenderEngine)
//// Both are called in ModelGraphBuilder (which makes sense for once)

class Shape : public ModelNode{
public:

  /// SETTERS
  void SetScale( double s ){
    Eigen::Vector3d scale(s, s, s);
    m_dScale = scale;
  }

  void SetScale( const Eigen::Vector3d& s ){
    m_dScale = s;
  }

  void SetMass( double dMass ){
    m_dMass = dMass;
  }

  void SetRestitution( double dRestitution ){
    m_dRestitution = dRestitution;
  }

  void SetPosition( double x, double y, double z){
    m_dPose[0] = x;
    m_dPose[1] = y;
    m_dPose[2] = z;
  }

  /// GETTERS
  Eigen::Vector3d GetScale(){
    return m_dScale;
  }

  double GetMass(){
    return m_dMass;
  }

  double GetRestitution(){
    return m_dRestitution;
  }

  /// Member variables
  Eigen::Vector3d m_dScale;
  double          m_dMass;
  double          m_dRestitution;

};

///////////////////////////////////////////////////////////////////////
///
/// ALL OF OUR SHAPES
///
///////////////////////////////////////////////////////////////////////

class BoxShape : public Shape
{
public:
  BoxShape(std::string sName, double x_length, double y_length, double z_length,
           double dMass, double dRestitution, std::vector<double> dPose){
    SetName(sName);
    SetMass(dMass);
    SetRestitution(dRestitution);
    SetPose(dPose);
    SetScale(1);
    m_dBounds<<x_length, y_length, z_length;
  }

  Eigen::Vector3d m_dBounds;

};

///////////////////


class CylinderShape : public Shape
{
public:
  CylinderShape(std::string sName, double dRadius, double dHeight,
                double dMass, double dRestitution,
                std::vector<double> dPose):
    m_dRadius(dRadius), m_dHeight(dHeight){
    SetName(sName);
    SetMass(dMass);
    SetRestitution(dRestitution);
    SetPose(dPose);
    SetScale(1);
  }

  double m_dRadius;
  double m_dHeight;
};


#endif // SHAPE_H

