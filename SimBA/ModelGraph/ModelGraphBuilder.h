#ifndef MODELGRAPHBUILDER_H
#define MODELGRAPHBUILDER_H

#include <ModelGraph/RenderEngine.h>
#include <ModelGraph/PhysicsEngine.h>

class ModelGraphBuilder{
public:

  enum BuildType{
    Physics = 1,
    Rendering = 2,
    All = 3
  };

  /////////////////////////////////////////
  /// FUNCTIONS
  /////////////////////////////////////////

  void AssociatePhysics(PhysicsEngine& m_Phys, ModelNode& item){
    m_Phys.RegisterObject( &item );
    Eigen::Vector6d ChildWorldPose;
    for(unsigned int ii = 0; ii < item.NumChildren(); ii++ ) {
      ChildWorldPose = _T2Cart(
            item.GetPoseMatrix() * item.m_vChildren[ii]->GetPoseMatrix());
      item.SetPose(ChildWorldPose);
      AssociatePhysics(m_Phys,*(item.m_vChildren[ii]));
    }
  }

  /////////////////////////////////////////

  void RenderGraph(Render& m_Render, ModelNode& item){
    m_Render.AddNode(&item);
    Eigen::Vector6d ChildWorldPose;
    for (unsigned int ii = 0; ii < item.NumChildren(); ii++ ) {
      ChildWorldPose = _T2Cart(
            item.GetPoseMatrix() * item.m_vChildren[ii]->GetPoseMatrix());
      item.SetPose(ChildWorldPose);
      RenderGraph(m_Render, *(item.m_vChildren[ii]));
    }
  }

  /////////////////////////////////////////

  void Init(int iBuildType, PhysicsEngine& m_Phys, Render& m_Render,
            ModelNode& m_RobotModel){
    iBuildType = iBuildType;
    if(iBuildType == Physics){
      AssociatePhysics(m_Phys, m_RobotModel);
    }
    else if(iBuildType == Rendering){
      RenderGraph(m_Render, m_RobotModel);
    }
    else if(iBuildType == All){
      AssociatePhysics(m_Phys, m_RobotModel);
      RenderGraph(m_Render, m_RobotModel);
    }
    else{
      cout<<"[ModelGraphBuilder] Fatal Error! Unknown Build Type!"<<endl;
    }
  }

  /////////////////////////////////////////

//  void PrintRobotGraph( ModelNode& mn ){
//    if (mn.m_pParent == NULL){
//      std::cout<<"I am "<<mn.GetName()<<" and I have "<<
//                 mn.NumChildren()<<" child(ren)."<<std::endl;
//    }
//    else{
//      std::cout<<"I am "<<mn.GetName()<<".  My parent is "<<
//                 mn.m_pParent->GetName()<<" and I have "<<mn.NumChildren()<<
//                 " child(ren)."<<std::endl;

//      if (dynamic_cast<Body*>(&mn) != NULL){
//        Body* pBody = (Body*)(&mn);
//        if (dynamic_cast<BoxShape*>(pBody->m_RenderShape) != NULL){
//          std::cout<<"I am a Box"<<std::endl;
//        }
//        else if (dynamic_cast<CylinderShape*>(pBody->m_RenderShape) != NULL){
//          std::cout<<"I am a Cylinder"<<std::endl;
//        }
//        std::cout<<"I am a Body"<<std::endl;
//      }
//      else if (HingeJoint* test = dynamic_cast<HingeJoint*>(&mn)){
//        std::cout<<"I am a HingeJoint"<<std::endl;
//      }
//      else if (HingeJoint* test = dynamic_cast<HingeJoint*>(&mn)){
//        std::cout<<"I am a Hinge2Joint"<<std::endl;
//      }
//      else{
//        std::cout << "I don't seem to exist." << std::endl;
//      }
//    }

//    for (unsigned int count = 0; count < mn.NumChildren(); count++ ){
//      PrintRobotGraph(*(mn.m_vChildren[count]));
//    }
//  }
};

#endif // MODELGRAPHBUILDER_H
