// manage all robots. Including user's robot and all other player's robot.
// By now we only support user to control one robot in his proxy. This robot will be considered as main robot.
// E.g. We should always create, access SimRobot via RobotManager.

#ifndef ROBOTSMANAGER_H
#define ROBOTSMANAGER_H


#include <iostream>
#include <vector>
#include <Eigen/Eigen>             // for vector maths
#include <Mvlpp/Mvl.h>
#include <Network/Messages.h>
#include <Robots/SimRobot.h>
#include <ModelGraph/PhyModelGraphAgent.h>    // for communicate between Physic Engine and Model Graph
#include <ModelGraph/RenderClass.h>
#include <tinyxml2.h>

using namespace std;
using namespace SceneGraph;
using namespace boost;


class RobotsManager
{
public:
    map<string, SimRobot*>            m_mSimRobotsList;  // the first robot join the list will be user's robot.
    WorldFullStateMsg                 m_WorldFullState;  // world state (pose, velocity) message. inlude the state of bodies.
    PhyModelGraphAgent                m_PhyMGAgent;
    Render                            m_Render;
    string                            m_sProxyName;
    string                            m_sMainRobotName;
    bool                              m_bStateKeeperOn;

    // ---------------------------------------------------------------------------------------------------------------------------
    // init RobotManager
    void Init(string sProxyName, string sServerName, PhyModelGraphAgent& rPhyMGAgent, Render& rRender)
    {
        m_PhyMGAgent = rPhyMGAgent;
        m_Render = rRender;
        m_sProxyName = sProxyName;

        if(sServerName == "WithoutStateKeeper" || sServerName =="WithoutNetwork")
        {
            m_bStateKeeperOn = false;
        }
        else
        {
            m_bStateKeeperOn = true;
        }
    }

    // ---------------------------------------------------------------------------------------------------------------------------
    // add robot with URDF file. The first robot add into the proxy will be considered as main robot (robot that player will control).
    // Otherwise it is other player's robot.
    bool AddRobot(XMLDocument& doc, string sProxyName)
    {
        SimRobot* pSimRobot = new SimRobot;
        string sRobotName;
        if( pSimRobot->Init(sProxyName, m_bStateKeeperOn, m_PhyMGAgent,m_Render,doc) != true)
        {
            cout<<"[RobotsManager] FatalError! Cannot init Robot!!"<<endl;
            return false;
        }
        else
        {
            // save this robot's name
            sRobotName = pSimRobot->GetRobotName();

            // check if we should save name of this robot as save main robot name
            if(m_mSimRobotsList.size()==0)
            {
                m_sMainRobotName = sRobotName;
            }
            m_mSimRobotsList.insert(pair<string, SimRobot*>(sRobotName, pSimRobot));
        }

        return true;
    }


    // ---------------------------------------------------------------------------------------------------------------------------
    // delete all body and joints of rigid body.
    // ************************ Notice! Sometimes the proxy will exit when we delete joint of robot. ***************************
    // ****** the error is exist in  ' for(; Hiter!=m_rPhyMGAgent.m_Agent.GetPhys()->m_HingeJointList.end(); Hiter++)  ' *******
    // **when we want to use iter of ' std::_Rb_tree_iterator<std::pair<std::string const, btHingeConstraint*> >::operator++' **
    void DeleteRobot(string sRobotName)
    {
        SimRobot* pSimRobot = GetRobot(sRobotName);
        if(pSimRobot->GetRobotName() == sRobotName)
        {
            GetRobot(sRobotName)->~SimRobot();
            std::map<string, SimRobot*>::iterator iter=  m_mSimRobotsList.find(sRobotName);
            m_mSimRobotsList.erase(iter);
            cout<<"[RobotsManager/DeleteRobot] Delete Robot :"<<sRobotName<<" success. Num of robot we have now is "<<m_mSimRobotsList.size()<<endl;
        }
        else
        {
            cout<<"[RobotManager/DeleteRobot] Cannot find robot "<<sRobotName<<endl;
        }
    }


    // ---------------------------------------------------------------------------------------------------------------------------
    // update full world state include all pose, command and velocity information of all bodies of main robot from each proxy
    void UpdateWorldFullState(WorldFullStateMsg worldfullstate)
    {
        m_WorldFullState = worldfullstate;
    }



    // ---------------------------------------------------------------------------------------------------------------------------
    // Simply draw all player on the screen
    void ApplyWorldFullStateOnAllPlayers()
    {
         DrawAllRobotsPoseAxis();
         ApplyWorldFullState();
    }


    // ---------------------------------------------------------------------------------------------------------------------------
    // Simplely apply the pose of all main robot of other proxy. DONOT apply the pose of This proxy's main robot.
    void ApplyWorldFullState()
    {
        for (int i=0;i!=m_WorldFullState.robot_state_size();i++)
        {
            RobotFullStateMsg mRobotState =  m_WorldFullState.robot_state(i);

            // only apply state of other robots
            string sRobotName = mRobotState.robot_name();

            if(sRobotName != GetMainRobot()->GetRobotName())
            {
                // get and apply the state of robot's all body
                for (int j=0;j!=mRobotState.mutable_body_state()->size();j++)
                {
                    BodyStateMsg* mBodyState = mRobotState.mutable_body_state(j);
                    string sBodyName = mBodyState->body_name();

                    // get velocity
                    Eigen::Vector3d eLinearVelocity;
                    eLinearVelocity<<  mBodyState->linear_velocity().x(), mBodyState->linear_velocity().y(), mBodyState->linear_velocity().z();
                    Eigen::Vector3d eAngularVelocity;
                    eAngularVelocity<< mBodyState->angular_velocity().x(), mBodyState->angular_velocity().y(), mBodyState->angular_velocity().z();

                    // get origin
                    Eigen::Vector3d eOrigin;
                    eOrigin << mBodyState->origin().x(), mBodyState->origin().y(), mBodyState->origin().z();

                    // get basis
                    Matrix33Msg origin = mBodyState->basis();
                    Eigen::Matrix3d mBasis;
                    mBasis<<origin.x11(), origin.x12(), origin.x13(),
                            origin.x21(), origin.x22(), origin.x23(),
                            origin.x31(), origin.x32(), origin.x33();

                    // apply in bullet engine
                    m_PhyMGAgent.m_Agent.SetEntityOrigin(sBodyName ,eOrigin);
                    m_PhyMGAgent.m_Agent.SetEntityBasis(sBodyName ,mBasis);
                    m_PhyMGAgent.m_Agent.SetEntityLinearvelocity(sBodyName, eLinearVelocity);
                    m_PhyMGAgent.m_Agent.SetEntityAngularvelocity(sBodyName, eAngularVelocity);
                }
            }
        }

    }



    /// draw robot in the screen
    // ---------------------------------------------------------------------------------------------------------------------------
    // Draw Axis is just one way of drawing players
    void DrawAllRobotsPoseAxis()
    {
//        for (int i=0;i!=m_WorldState.robots_size();i++)
//        {

//                Eigen::Vector6d pose;
//                pose<<  m_WorldState.robots(i).pose().x(),
//                        m_WorldState.robots(i).pose().y(),
//                        m_WorldState.robots(i).pose().z(),
//                        m_WorldState.robots(i).pose().p(),
//                        m_WorldState.robots(i).pose().q(),
//                        m_WorldState.robots(i).pose().r();// axis edge

//               Eigen::Vector6d AxisX, AxisY, AxisZ;

//               GenPoseAxis(pose,AxisX,AxisY,AxisZ);

//               glBegin(GL_LINES);
//               glColor3f(1.0f,0.0f,0.0f);
//               glVertex3f(m_WorldState.robots(i).pose().x(),m_WorldState.robots(i).pose().y(),m_WorldState.robots(i).pose().z());//origin x
//               glVertex3f(AxisX(0,0),AxisX(1,0),AxisX(2,0));

//               glColor3f(0.0f,1.0f,0.0f);
//               glVertex3f(m_WorldState.robots(i).pose().x(),m_WorldState.robots(i).pose().y(),m_WorldState.robots(i).pose().z());//origin y
//               glVertex3f(AxisY(0,0),AxisY(1,0),AxisY(2,0));

//               glColor3f(0.0f,0.0f,1.0f);
//               glVertex3f(m_WorldState.robots(i).pose().x(),m_WorldState.robots(i).pose().y(),m_WorldState.robots(i).pose().z());//origin z
//               glVertex3f(AxisZ(0,0),AxisZ(1,0),AxisZ(2,0));
//               glEnd();

//               glColor3f(1.0f, 0.0f, 0.0f);
//               glRasterPos3f(m_WorldState.robots(i).pose().x(),m_WorldState.robots(i).pose().y(),m_WorldState.robots(i).pose().z()+1);
////               glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char*)m_WorldState.robots(i).robot_name().c_str());
//        }
    }



    // ---------------------------------------------------------------------------------------------------------------------------
    // GenAxis x,y,z for Pose (x,y,z,p,q,r)
    void GenPoseAxis(Eigen::Vector6d &Pose, Eigen::Vector6d &AxisX, Eigen::Vector6d &AxisY, Eigen::Vector6d &AxisZ)
    {
         int length=1;
         double origin_x=Pose(0,0);
         double origin_y=Pose(1,0);
         double origin_z=Pose(2,0);
         double p=Pose(3,0);
         double q=Pose(4,0);
         double r=Pose(5,0);

         AxisX<<origin_x+length*cos(q)*cos(r), origin_y+length*(cos(r)*sin(p)*sin(q) + cos(p)*sin(r)), origin_z+length*(sin(p)*sin(r) - cos(p)*cos(r)*sin(q)), 0, 0, 0;
         AxisY<<origin_x+length*(-cos(q)*sin(r)),  origin_y+length*( cos(p)*cos(r) - sin(p)*sin(q)*sin(r)), origin_z+length*( cos(p)*sin(q)*sin(r) + cos(r)*sin(p)), 0, 0, 0;
         AxisZ<<origin_x+length*(sin(q)),  origin_y+length*(-cos(q)*sin(p)), origin_z+length*(cos(p)*cos(q)), 0, 0, 0;
     }



    /// Get pointer of a robot
    // ---------------------------------------------------------------------------------------------------------------------------
    // get robot that one proxy actually can control (main robot)
    SimRobot* GetMainRobot()
    {
        string sRobotName = m_sMainRobotName;
        SimRobot* pSimRobot = m_mSimRobotsList.find(sRobotName)->second;
        return pSimRobot;
    }


    // ---------------------------------------------------------------------------------------------------------------------------
    // get any robot in the proxy, include other player's robot. user should not use this function to get main robot
    SimRobot* GetRobot(string sRobotName)
    {
        SimRobot* pSimRobot = m_mSimRobotsList.find(sRobotName)->second;
        return pSimRobot;
    }


};

#endif
