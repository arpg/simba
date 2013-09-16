#ifndef KEYCONTROL_H
#define KEYCONTROL_H

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>               // for reference counting pointers
#include <ModelGraph/PhysicsClass.h>



class Keycontrol
{
public:
    Phys*                      m_Phys;



    ///////////////////////////////////////////////////////////////////
    void LeftKey()
    {
        m_Phys->ApplySteeringToEntity("BRWheel",100);
        m_Phys->ApplySteeringToEntity("BLWheel",100);
    }

    ///////////////////////////////////////////////////////////////////
    void RightKey()
    {
        m_Phys->ApplySteeringToEntity("BRWheel",-100);
        m_Phys->ApplySteeringToEntity("BLWheel",-100);
    }

    ///////////////////////////////////////////////////////////////////
    void ForwardKey()
    {
//            m_Phys->ApplyForceToEntity("BRWheel",1000);
//            m_Phys->ApplyForceToEntity("BLWheel",1000);
        m_Phys->ApplyForceToEntity("Chassis",100000);

//            m_Phys->ApplyTorque("FRWheel",200);
//            m_Phys->ApplyTorque("FLWheel",200);
//            cout<< m_Phys->GetEntityRotation("FRWheel")<<endl;
//            m_Phys->SetEntityPoseLinear("Chassis",-2000);
    }

    ///////////////////////////////////////////////////////////////////
    void ReverseKey()
    {
//            m_Phys->ApplyForceToEntity("FRWheel",-3000);
//            m_Phys->ApplyForceToEntity("FLWheel",-3000);
        m_Phys->ApplyForceToEntity("Chassis",-100000);

//            m_Phys->ApplyTorque("FRWheel",-200);
//            m_Phys->ApplyTorque("FLWheel",-200);
    }

    // ---- roll
    void IncreaseCamRoll()
    {
        double roll,pitch,yaw;
        m_Phys->GetEntityRotation("RCamera",roll,pitch,yaw);

        roll = roll+0.2;
        m_Phys->SetEntityRotation("RCamera",roll,pitch,yaw);
    }

    void DecreaseCamRoll()
    {
        double roll,pitch,yaw;
        m_Phys->GetEntityRotation("RCamera",roll,pitch,yaw);

        roll = roll-0.2;
        m_Phys->SetEntityRotation("RCamera",roll,pitch,yaw);
    }

    // ---- pitch
    void IncreaseCamPitch()
    {
        double roll,pitch,yaw;
        m_Phys->GetEntityRotation("RCamera",roll,pitch,yaw);

        pitch = pitch+0.2;
        m_Phys->SetEntityRotation("RCamera",roll,pitch,yaw);
    }

    void DecreaseCamPitch()
    {
        double roll,pitch,yaw;
        m_Phys->GetEntityRotation("RCamera",roll,pitch,yaw);

        pitch = pitch-0.2;
        m_Phys->SetEntityRotation("RCamera",roll,pitch,yaw);
    }

    // ---- Yaw
    void IncreaseCamYaw()
    {
        double roll,pitch,yaw;
        m_Phys->GetEntityRotation("RCamera",roll,pitch,yaw);

        yaw = yaw+0.2;
        m_Phys->SetEntityRotation("RCamera",roll,pitch,yaw);
    }

    void DecreaseCamYaw()
    {
        double roll,pitch,yaw;
        m_Phys->GetEntityRotation("RCamera",roll,pitch,yaw);

        yaw = yaw-0.2;
        m_Phys->SetEntityRotation("RCamera",roll,pitch,yaw);
    }





};


#endif // KEYCONTROL_H
