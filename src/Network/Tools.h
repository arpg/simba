#ifndef TOOLS_H
#define TOOLS_H

//#include <Network/Messages.h>
//#include <Robots/RobotsManager.h>

//// ---------------------------------------------------------------------------------------------------------------------------
//// return full state message of robot. Include Pose, Velocity (linear and angular)
//// Generally speaking, this will be the state of our main robot.
//RobotFullStateMsg GetMainRobotFullState(int iTimeStep, RobotsManager& rRobotManager)
//{
//    RobotFullStateMsg mRobotState;
//    // 1. set general information
//        string sMainRobotName =rRobotManager.GetMainRobot()->GetRobotName();
//        mRobotState.set_robot_name(sMainRobotName);
//        mRobotState.set_time_step(iTimeStep);

//        vector<string> vAllBodyFullName = rRobotManager.GetMainRobot()->GetAllBodyName();

//    // 2. set body state info
//        for (int i=0;i!=vAllBodyFullName.size();i++)
//        {
//            string sBodyName = vAllBodyFullName[i];
//            // prepare pose info
//            double x,y,z,p,q,r;
//            rRobotManager.m_PhyMGAgent.m_Agent.GetEntity6Pose(sBodyName,x,y,z,p,q,r);

//            // prepare veloicty info
//            Eigen::Vector3d eLinearV = rRobotManager.m_PhyMGAgent.m_Agent.GetEntityLinearVelocity(sBodyName);
//            Eigen::Vector3d eAngularV = rRobotManager.m_PhyMGAgent.m_Agent.GetEntityAngularVelocity(sBodyName);

//            // set body information
//            BodyStateMsg* mBodyState = mRobotState.add_body_state();
//            mBodyState->set_name(sBodyName);
//            mBodyState->mutable_pose()->set_x(x);
//            mBodyState->mutable_pose()->set_y(y);
//            mBodyState->mutable_pose()->set_z(z);
//            mBodyState->mutable_pose()->set_p(p);
//            mBodyState->mutable_pose()->set_q(q);
//            mBodyState->mutable_pose()->set_r(r);

//            mBodyState->mutable_linear_velocity()->set_x(eLinearV[0]);
//            mBodyState->mutable_linear_velocity()->set_y(eLinearV[1]);
//            mBodyState->mutable_linear_velocity()->set_z(eLinearV[2]);

//            mBodyState->mutable_angular_velocity()->set_x(eAngularV[0]);
//            mBodyState->mutable_angular_velocity()->set_y(eAngularV[1]);
//            mBodyState->mutable_angular_velocity()->set_z(eAngularV[2]);
//        }

//    // 3. set command info
//        Eigen::Vector6d eCommand;
//        vector<string> vBodyFullName;
//        rRobotManager.GetMainRobot()->m_Controller.GetLatestCommandAndBodyFullName(vBodyFullName, eCommand);
//        mRobotState.mutable_command()->set_time_step(iTimeStep);
//        mRobotState.mutable_command()->set_x( eCommand[0] );
//        mRobotState.mutable_command()->set_y( eCommand[1] );
//        mRobotState.mutable_command()->set_z( eCommand[2] );
//        mRobotState.mutable_command()->set_p( eCommand[3] );
//        mRobotState.mutable_command()->set_q( eCommand[4] );
//        mRobotState.mutable_command()->set_r( eCommand[5] );

//        // set full name of body that will be apply this command
//        for (int i=0; i != vBodyFullName.size();i++)
//        {
//            mRobotState.mutable_command()->add_body_name()->assign(vBodyFullName[i]);
//        }

//    return mRobotState;
//}



////WorldState ConvertWorldStateMsg(WorldStateMsg mWorldState)
////{
////    WorldState newWorldState;
////    newWorldState.m_iTimeStep = mWorldState.time_step();

////    for (int i=0;i!=mWorldState.robots_size();i++)
////    {
////        RobotState newRobotState;
////        newRobotState.m_sRobotName = mWorldState.robots(i).robot_name();

////        Eigen::Vector6d pose;
////        pose<<  mWorldState.robots(i).pose().x(),
////                mWorldState.robots(i).pose().y(),
////                mWorldState.robots(i).pose().z(),
////                mWorldState.robots(i).pose().p(),
////                mWorldState.robots(i).pose().q(),
////                mWorldState.robots(i).pose().r();// axis edge

////        newRobotState.m_Pose = pose;

////        Eigen::Vector6d command;
////        command<< mWorldState.robots(i).command().x(),
////                  mWorldState.robots(i).command().y(),
////                  mWorldState.robots(i).command().z(),
////                  mWorldState.robots(i).command().p(),
////                  mWorldState.robots(i).command().q(),
////                  mWorldState.robots(i).command().r();

////        newRobotState.m_Command = command;

////        newWorldState.m_vWorldState.push_back(newRobotState);
////    }

////    return newWorldState;
////}

#endif // TOOLS_H
