#ifndef MODELGRAPHBUILDER_H
#define MODELGRAPHBUILDER_H

#include <ModelGraph/Models.h>
#include <ModelGraph/RenderClass.h>
#include <ModelGraph/PhysicsClass.h>


class ModelGraphBuilder{
    ///////////////////////////////////////////////////////////////////
public:
    int         m_iBuildType;

    void Init(int iBuildType,Phys* m_Phys, Render& m_Render,ModelNode& m_RobotModel,Eigen::Vector6d m_RobotPose)
    {
        m_iBuildType = iBuildType;

        if(m_iBuildType == BuildPhysic)
        {
            AssociatePhysicsBodies(m_Phys, m_RobotModel, m_RobotPose);
            AssociatePhysicsJoints(m_Phys, m_RobotModel, m_RobotPose);
        }
        else if(m_iBuildType == BuildRender)
        {
            BuildRenderGraph(m_Render,m_RobotModel,m_RobotPose);
        }
        else if(m_iBuildType == BuildAll)
        {
            AssociatePhysicsBodies(m_Phys, m_RobotModel, m_RobotPose);
            AssociatePhysicsJoints(m_Phys, m_RobotModel, m_RobotPose);
            BuildRenderGraph(m_Render,m_RobotModel,m_RobotPose);
        }
        else
        {
            cout<<"[ModelGraphBuilder] Fatal Error! Unknown Build Type!"<<endl;
        }
    }


    void AssociatePhysicsBodies(Phys* m_Phys, ModelNode& item, Eigen::Vector6d WorldPose )
    {
        if (dynamic_cast<Body*>(&item))
            m_Phys->RegisterObject( &item, item.GetName(), WorldPose);
        Eigen::Vector6d ChildWorldPose;
        for (unsigned int count = 0; count < item.NumChildren(); count++ ) {
            ChildWorldPose = _T2Cart(_Cart2T(WorldPose)*(item.m_vChildren[count]->GetPoseMatrix()));
            AssociatePhysicsBodies(m_Phys,*(item.m_vChildren[count]), ChildWorldPose);
        }
    }

    ///////////////////////////////////////////////////////////////////
    void AssociatePhysicsJoints(Phys* m_Phys, ModelNode& item, Eigen::Vector6d WorldPose )
    {
        if (dynamic_cast<Joint*>(&item))
            m_Phys->RegisterObject( &item, item.GetName(), WorldPose);
        Eigen::Vector6d ChildWorldPose;
        for (unsigned int count = 0; count < item.NumChildren(); count++ ) {
            ChildWorldPose = _T2Cart(_Cart2T(WorldPose)*(item.m_vChildren[count]->GetPoseMatrix()));
            AssociatePhysicsJoints(m_Phys, *(item.m_vChildren[count]), ChildWorldPose);
        }
    }

    ///////////////////////////////////////////////////////////////////
    void BuildRenderGraph(Render& m_Render, ModelNode& item, Eigen::Vector6d WorldPose )
    {
        m_Render.AddNode(&item, WorldPose);

        Eigen::Vector6d ChildWorldPose;
        for (unsigned int count = 0; count < item.NumChildren(); count++ ) {
            ChildWorldPose = _T2Cart(_Cart2T(WorldPose)*(item.m_vChildren[count]->GetPoseMatrix()));
            BuildRenderGraph(m_Render, *(item.m_vChildren[count]), ChildWorldPose);
        }
    }

    ///////////////////////////////////////////////////////////////////
    void PrintRobotGraph( ModelNode& mn )
    {
        if (mn.m_pParent == NULL)
        {
                std::cout<<"I am "<<mn.GetName()<<" and I have "<<mn.NumChildren()<<" child(ren)."<<std::endl;
        }
        else{
                std::cout<<"I am "<<mn.GetName()<<".  My parent is "<<mn.m_pParent->GetName()<<" and I have "<<mn.NumChildren()<<" child(ren)."<<std::endl;

            if (dynamic_cast<Body*>(&mn) != NULL)
            {
                Body* pBody = (Body*)(&mn);
                if (dynamic_cast<BoxShape*>(pBody->m_RenderShape) != NULL)
                {
                    std::cout<<"I am a Box"<<std::endl;
                }
                else if (dynamic_cast<CylinderShape*>(pBody->m_RenderShape) != NULL)
                {
                    std::cout<<"I am a Cylinder"<<std::endl;
                }
                std::cout<<"I am a Body"<<std::endl;
            }
            else if (HingeJoint* test = dynamic_cast<HingeJoint*>(&mn))
            {
                std::cout<<"I am a HingeJoint"<<std::endl;
            }
            else if (HingeJoint* test = dynamic_cast<HingeJoint*>(&mn))
            {
                std::cout<<"I am a Hinge2Joint"<<std::endl;
            }
            else
            {
                std::cout << "I don't seem to exist." << std::endl;
            }
        }

        for (unsigned int count = 0; count < mn.NumChildren(); count++ )
        {
            PrintRobotGraph(*(mn.m_vChildren[count]));
        }
    }

    enum BuildType{
         BuildPhysic = 1,
         BuildRender = 2,
         BuildAll = 3
    };
};

#endif // MODELGRAPHBUILDER_H
