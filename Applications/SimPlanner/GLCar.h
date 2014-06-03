#ifndef _GL_CAR_
#define _GL_CAR_

#include <Mvlpp/SE3.h>
#include "RigidBody.h"

#include "pangolin/pangolin.h"
#include "SceneGraph/SceneGraph.h"
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

using namespace SceneGraph;
using namespace pangolin;

// TODO: FIX GL_CAR TO SEE WHAT CAR IS DOING.

class GLCar {
public:

  GLCar() {
    draw_scheme_ = "lines";
    visible_ = false;
  }

  void Init(double wheelbase, double width, double height,
            double wheel_radius, double wheel_width,
            SceneGraph::GLSceneGraph& scenegraph) {
    visible_ = true;
    body_mesh_dir_ = "NULL";
    wheel_mesh_dir_ = "NULL";
    color_ = GLColor();
    chassis_ = new SceneGraph::GLBox();
    chassis_->SetExtent(wheelbase, width, height);
    chassis_->SetPerceptable(true);
    scenegraph.AddChild(chassis_);
    //load the wheels
    wheels_.resize(4);
    for (size_t i = 0; i < wheels_.size(); i++) {
      wheels_[i] = new GLCylinder();
      wheels_[i]->Init(wheel_radius, wheel_radius, wheel_width, 10, 10);
      wheels_[i]->SetPerceptable(true);
      scenegraph.AddChild(wheels_[i]);
    }
  }

  void Init(std::string body_mesh_dir, std::string wheel_mesh_dir,
            SceneGraph::GLSceneGraph& scenegraph) {
    draw_scheme_ = "mesh";
    visible_ = true;
    body_mesh_dir_ = body_mesh_dir;
    wheel_mesh_dir_ = wheel_mesh_dir;
    color_ = GLColor();
    //only if the body isn't a triangle, load the meshes
    body_mesh_ = new GLMesh();
    body_mesh_->Init(body_mesh_dir_);
    body_mesh_->SetPerceptable(true);
    body_mesh_->SetVisible(true);
    scenegraph.AddChild(body_mesh_);
    //load the wheels
    wheel_meshes_.resize(4);
    for (size_t i = 0; i < wheel_meshes_.size(); i++) {
      wheel_meshes_[i] = new GLMesh();
      wheel_meshes_[i]->Init(wheel_mesh_dir_);
      wheel_meshes_[i]->SetPerceptable(true);
      scenegraph.AddChild(wheel_meshes_[i]);
    }
  }

  void SetCarPose(const Eigen::Matrix4d& pose) {
    if (draw_scheme_=="mesh") {
      body_mesh_->SetPose(pose);
    } else {
      chassis_->SetPose(pose);
    }
    m_Pose = pose;
  }

  void SetWheelPose(const unsigned int& wheel_int,
                    const Eigen::Matrix4d& pose) {
    Eigen::Vector6d new_pose = SwitchWheelYaw(T2Cart(pose));
    if (draw_scheme_=="mesh") {
      wheel_meshes_[wheel_int]->SetPose(new_pose);
    } else {
      wheels_[wheel_int]->SetPose(new_pose);
    }
  }

  void SetColor(GLColor C){
    color_ = C;
  }

  void SetVisible(bool visible){
    if (draw_scheme_=="mesh") {
      body_mesh_->SetVisible(visible);
      for (size_t i = 0; i < wheel_meshes_.size(); i++) {
        wheel_meshes_[i]->SetVisible(visible);
      }
    }
    visible_ = visible;
  }

  void SetCarScale(Eigen::Vector3d scale){
    if (draw_scheme_=="mesh") {
      body_mesh_->SetScale(scale);
    }
  }

  void SetWheelScale(Eigen::Vector3d scale){
    if (draw_scheme_=="mesh") {
      for (size_t i = 0; i < wheel_meshes_.size(); i++) {
        wheel_meshes_[i]->SetScale(scale);
      }
    }
  }

  GLObject* GetBody(){
    return body_mesh_;
  }

  std::vector<GLMesh *>& GetWheels() {
    return wheel_meshes_;
  }

  Eigen::Matrix4d GetPose4x4_po(){
    return m_Pose;
  }

  Eigen::Vector6d SwitchWheelYaw(Eigen::Vector6d bad_yaw){
    Eigen::Vector6d good_yaw;
    good_yaw<<bad_yaw(0), bad_yaw(1), bad_yaw(2),
        bad_yaw(4), -bad_yaw(3), bad_yaw(5);
    Eigen::Vector6d temp;
    temp<<0,0,0,M_PI/2,0,0;
    good_yaw = good_yaw+temp;
    return good_yaw;
  }

  bool IsVisible(){
    return visible_;
  }

 protected:
  // centroid position + orientation
  Eigen::Matrix4d m_Pose;
  // control scale of car
  GLColor color_;
  bool visible_;
  std::string body_mesh_dir_;
  std::string wheel_mesh_dir_;
  SceneGraph::GLMesh* body_mesh_;
  std::vector<SceneGraph::GLMesh *> wheel_meshes_;
  SceneGraph::GLBox* chassis_;
  std::vector<SceneGraph::GLCylinder *> wheels_;
  std::string draw_scheme_;

};


#endif	/* _GL_CAR_ */
