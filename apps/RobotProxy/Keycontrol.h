#ifndef KEYCONTROL_H
#define KEYCONTROL_H

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>               // for reference counting pointers
#include <ModelGraph/PhysicsClass.h>



class Keycontrol
{
public:
    Phys*                      m_Phys;


    // the following apply troque and steering to wheels in order to control robot.
//        //----------------------------------------------------------------------------------------------------------------------------------------------
//        void LeftKey()
//        {
//            Eigen::Vector6d  dCommand;
//            vector<string> vBodyFullName;
//            string sMainRobotName = m_pMainRobot->GetRobotName();

//            dCommand<<0, 0, 0, 0, 0, -800;
//            vBodyFullName.push_back("FRWheel@"+sMainRobotName);
//            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);

//            dCommand<<0, 0, 0, 0, 0, -800;
//            vBodyFullName.push_back("FLWheel@"+sMainRobotName);
//            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);

//            m_SimDeviceManager.GetSimpleController("MController")->ApplyCommand();
//        }

//        ///////////////////////////////////////////////////////////////////
//        void RightKey()
//        {
//            Eigen::Vector6d  dCommand;
//            vector<string> vBodyFullName;
//            string sMainRobotName = m_pMainRobot->GetRobotName();

//            dCommand<<0, 0, 0, 0, 0, 800;
//            vBodyFullName.push_back("FRWheel@"+sMainRobotName);
//            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);

//            dCommand<<0, 0, 0, 0, 0, 800;
//            vBodyFullName.push_back("FLWheel@"+sMainRobotName);
//            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);

//            m_SimDeviceManager.GetSimpleController("MController")->ApplyCommand();
//        }

//        ///////////////////////////////////////////////////////////////////
//        void ForwardKey()
//        {
//            Eigen::Vector6d  dCommand;
//            vector<string> vBodyFullName;
//            string sMainRobotName = m_pMainRobot->GetRobotName();

//            dCommand<<0, -8000, 0,0,0,0;
//            vBodyFullName.push_back("BRWheel@"+sMainRobotName);
//            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);

//            dCommand<<0, 8000, 0,0,0,0;
//            vBodyFullName.push_back("BLWheel@"+sMainRobotName);
//            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);

//            m_SimDeviceManager.GetSimpleController("MController")->ApplyCommand();

//        }

//        ///////////////////////////////////////////////////////////////////
//        void ReverseKey()
//        {
//            Eigen::Vector6d  dCommand;
//            vector<string> vBodyFullName;
//            string sMainRobotName = m_pMainRobot->GetRobotName();

//            dCommand<<0, 4000, 0,0,0,0;
//            vBodyFullName.push_back("BRWheel@"+sMainRobotName);
//            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);

//            dCommand<<0, -4000, 0,0,0,0;
//            vBodyFullName.push_back("BLWheel@"+sMainRobotName);
//            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);

//            m_SimDeviceManager.GetSimpleController("MController")->ApplyCommand();
//        }

/// ==================================================================================================================================


    // the following is old version camera control keys
//    // ---- roll
//    void IncreaseCamRoll()
//    {
//        double roll,pitch,yaw;
//        m_Phys->GetEntityRotation("RCamera",roll,pitch,yaw);

//        roll = roll+0.2;
//        m_Phys->SetEntityRotation("RCamera",roll,pitch,yaw);
//    }

//    void DecreaseCamRoll()
//    {
//        double roll,pitch,yaw;
//        m_Phys->GetEntityRotation("RCamera",roll,pitch,yaw);

//        roll = roll-0.2;
//        m_Phys->SetEntityRotation("RCamera",roll,pitch,yaw);
//    }

//    // ---- pitch
//    void IncreaseCamPitch()
//    {
//        double roll,pitch,yaw;
//        m_Phys->GetEntityRotation("RCamera",roll,pitch,yaw);

//        pitch = pitch+0.2;
//        m_Phys->SetEntityRotation("RCamera",roll,pitch,yaw);
//    }

//    void DecreaseCamPitch()
//    {
//        double roll,pitch,yaw;
//        m_Phys->GetEntityRotation("RCamera",roll,pitch,yaw);

//        pitch = pitch-0.2;
//        m_Phys->SetEntityRotation("RCamera",roll,pitch,yaw);
//    }

//    // ---- Yaw
//    void IncreaseCamYaw()
//    {
//        double roll,pitch,yaw;
//        m_Phys->GetEntityRotation("RCamera",roll,pitch,yaw);

//        yaw = yaw+0.2;
//        m_Phys->SetEntityRotation("RCamera",roll,pitch,yaw);
//    }

//    void DecreaseCamYaw()
//    {
//        double roll,pitch,yaw;
//        m_Phys->GetEntityRotation("RCamera",roll,pitch,yaw);

//        yaw = yaw-0.2;
//        m_Phys->SetEntityRotation("RCamera",roll,pitch,yaw);
//    }


    /// ==================================================================================================================================

    // the following is new version of camera control keys
    // ---- roll
    void IncreaseCamRoll()
    {
//            double roll,pitch,yaw;
//            roll = 0.1;
//            pitch = 0;
//            yaw = 0;
//              m_SimRobot->m_Controller.SetRotation("RCameraRGB",roll,pitch,yaw);
    }

    void DecreaseCamRoll()
    {
//            double roll =-0.1;
//            double pitch = 0;
//            double yaw =0;
//            m_SimRobot->m_Controller.SetRotation("RCameraRGB",roll,pitch,yaw);
    }

    // ---- pitch
    void IncreaseCamPitch()
    {
//           double roll,pitch,yaw;
//            roll = 0;
//            pitch = 0.1;
//            yaw = 0;
//            m_SimRobot->m_Controller.SetRotation("RCameraRGB",roll,pitch,yaw);
    }

    void DecreaseCamPitch()
    {
//            double roll,pitch,yaw;
//            roll = 0;
//            pitch = -0.1;
//            yaw = 0;
//            m_SimRobot->m_Controller.SetRotation("RCameraRGB",roll,pitch,yaw);
    }

    // ---- Yaw
    void IncreaseCamYaw()
    {
//            double roll,pitch,yaw;
//            roll = 0;
//            pitch = 0;
//            yaw = 0.1;
        cout<<"you press increase yaw"<<endl;
//            m_SimRobot->m_Controller.SetRotation("RCameraRGB",roll,pitch,yaw);
    }

    void DecreaseCamYaw()
    {
//            double roll,pitch,yaw;
//            roll = 0;
//            pitch = 0;
//            yaw = -0.1;
//            cout<<"you press decrease yaw"<<endl;
//            m_SimRobot->m_Controller.SetRotation("RCameraRGB",roll,pitch,yaw);
    }

    /// ==================================================================================================================================
};


#endif // KEYCONTROL_H
