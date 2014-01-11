// this is a bullet PhysicsEngine warpper.

#ifndef BULLETMODELGRAPHAGENT_H
#define BULLETMODELGRAPHAGENT_H

#include <ModelGraph/PhysicsEngine.h>
//#include <bullet/BulletWorldImporter/btBulletWorldImporter.h>

class BulletModelGraphAgent
{
public:
    BulletModelGraphAgent()
    {

    }


    // --------------------------------------------------------------------------------------------------------------
    bool Init()
    {
        m_pPhys = new Phys;
        m_pPhys->Init();
        return true;
    }



    // --------------------------------------------------------------------------------------------------------------
    Phys* GetPhys()
    {
        return m_pPhys;
    }



    // --------------------------------------------------------------------------------------------------------------
    // get pose of entity. This function is experimental
    Eigen::Vector6d GetEntity6Pose( string name )
    {
        Entity e = m_pPhys->getEntity( name );
        btRigidBody* rB = e.m_pRigidBody.get();

        btTransform WorldTransform = rB->getCenterOfMassTransform();

        btScalar roll, pitch, yaw;
        WorldTransform.getBasis().getEulerZYX(yaw,pitch,roll);
        double x, y, z, r, p, q;
        x = WorldTransform.getOrigin().getX();
        y = WorldTransform.getOrigin().getY();
        z = WorldTransform.getOrigin().getZ();
        r = roll;
        p = pitch;
        q = yaw;
        Eigen::Vector6d toRet;
        toRet << x, y, z, r, p, q;

        return toRet;
    }

    void GetEntity6Pose(string name, Eigen::Vector6d& rPose)
    {
        Entity e = m_pPhys->getEntity(name);
        btRigidBody* rB = e.m_pRigidBody.get();

        btTransform WorldTransform = rB->getCenterOfMassTransform();

        btScalar roll, pitch, yaw;
        WorldTransform.getBasis().getEulerZYX(yaw,pitch,roll);

        rPose[0] = WorldTransform.getOrigin().getX();
        rPose[1] = WorldTransform.getOrigin().getY();
        rPose[2] = WorldTransform.getOrigin().getZ();
        rPose[3] = roll;
        rPose[4] = pitch;
        rPose[5] = yaw;
    }



    void SetEntity6Pose(string sName, Eigen::Vector6d Pose)
    {
        SetEntityRotation(sName, Pose(3,0), Pose(4,0), Pose(5,0));

        Eigen::Vector3d eOrigin;
        eOrigin<<Pose(0,0), Pose(1,0), Pose(2,0);
        SetEntityOrigin(sName, eOrigin);
    }


    // --------------------------------------------------------------------------------------------------------------
    Eigen::Vector3d GetEntityOrigin(string sName)
    {

        Entity e = m_pPhys->getEntity(sName);
        btRigidBody* rB = e.m_pRigidBody.get();

        btVector3 btOrigin = rB->getCenterOfMassTransform().getOrigin();

        Eigen::Vector3d eOrigin;
        eOrigin<< btOrigin.getX(), btOrigin.getY() , btOrigin.getZ();

        return eOrigin;
    }

    // --------------------------------------------------------------------------------------------------------------
        Eigen::Matrix3d GetEntityBasis(string sName)
    {
        Entity e = m_pPhys->getEntity(sName);
        btRigidBody* rB = e.m_pRigidBody.get();

        btMatrix3x3 btBasis = rB->getCenterOfMassTransform().getBasis();

        Eigen::Matrix3d mBasis;
        mBasis << btBasis[0][0], btBasis[0][1], btBasis[0][2],
                  btBasis[1][0], btBasis[1][1], btBasis[1][2],
                  btBasis[2][0], btBasis[2][1], btBasis[2][2];
        return mBasis;
    }




    // --------------------------------------------------------------------------------------------------------------
    void SetEntityOrigin(string sName, Eigen::Vector3d eOrigin)
    {
        Entity e = m_pPhys->getEntity(sName);
        btRigidBody* rB = e.m_pRigidBody.get();

        btTransform btTran = rB->getCenterOfMassTransform();

        btVector3 BtOrigin = btTran.getOrigin();
        BtOrigin.setX(eOrigin[0]);
        BtOrigin.setY(eOrigin[1]);
        BtOrigin.setZ(eOrigin[2]);
        btTran.setOrigin(BtOrigin);

        rB->setCenterOfMassTransform(btTran);
    }



    // --------------------------------------------------------------------------------------------------------------
    void SetEntityBasis(string sName, Eigen::Matrix3d mBasis)
    {
        Entity e = m_pPhys->getEntity(sName);
        btRigidBody* rB = e.m_pRigidBody.get();

        btTransform btTran = rB->getCenterOfMassTransform();

        btMatrix3x3 btBasis;
        btBasis.setValue(mBasis(0,0),mBasis(0,1),mBasis(0,2),mBasis(1,0),mBasis(1,1),mBasis(1,2),mBasis(2,0),mBasis(2,1),mBasis(2,2));
        btTran.setBasis(btBasis);

        rB->setCenterOfMassTransform(btTran);
    }



    // --------------------------------------------------------------------------------------------------------------
    Eigen::Vector3d GetEntityLinearVelocity(string sBodyFullName)
    {
        Entity e = m_pPhys->getEntity(sBodyFullName);
        btRigidBody* rB = e.m_pRigidBody.get();

        btVector3 LinVelocity = rB->getLinearVelocity();

        Eigen::Vector3d  eLinearVelocity;
        eLinearVelocity<<LinVelocity.getX(),LinVelocity.getY(),LinVelocity.getZ();

        return eLinearVelocity;
    }


    // --------------------------------------------------------------------------------------------------------------
    double GetEntityVelocity(string name)
    {
        Entity e = m_pPhys->getEntity(name);
        btRigidBody* rB = e.m_pRigidBody.get();

        double vx = rB->getLinearVelocity()[0];
        double vy = rB->getLinearVelocity()[1];
        double vz = rB->getLinearVelocity()[2];

        double velocity = sqrt(vx*vx + vy*vy+vz*vz);
        return velocity;
    }


    // --------------------------------------------------------------------------------------------------------------
    Eigen::Vector3d GetEntityAngularVelocity(string sBodyFullName)
    {
        Entity e = m_pPhys->getEntity(sBodyFullName);
        btRigidBody* rB = e.m_pRigidBody.get();

        btVector3 AngVelocity = rB->getAngularVelocity();

        Eigen::Vector3d  eAngularVelocity;
        eAngularVelocity<<AngVelocity.getX(),AngVelocity.getY(),AngVelocity.getZ();

        return eAngularVelocity;
    }


    // --------------------------------------------------------------------------------------------------------------
    void SetEntityLinearvelocity(string sBodyFullName, Eigen::Vector3d eLinearVelocity)
    {
        Entity e = m_pPhys->getEntity(sBodyFullName);
        btRigidBody* rB = e.m_pRigidBody.get();

        btVector3 btLinearvelocity;
        btLinearvelocity.setX(eLinearVelocity[0]);
        btLinearvelocity.setY(eLinearVelocity[1]);
        btLinearvelocity.setZ(eLinearVelocity[2]);

        rB->setLinearVelocity(btLinearvelocity);
    }


    // --------------------------------------------------------------------------------------------------------------
    void SetEntityAngularvelocity(string sBodyFullName, Eigen::Vector3d eAngularVelocity)
    {
        Entity e = m_pPhys->getEntity(sBodyFullName);
        btRigidBody* rB = e.m_pRigidBody.get();

        btVector3 btAngularvelocity;
        btAngularvelocity.setX(eAngularVelocity[0]);
        btAngularvelocity.setY(eAngularVelocity[1]);
        btAngularvelocity.setZ(eAngularVelocity[2]);

        rB->setAngularVelocity(btAngularvelocity);
    }


    // --------------------------------------------------------------------------------------------------------------
    void SetEntityRotation(string EntityName, double roll, double pitch, double yaw)
    {
        Entity e = m_pPhys->getEntity(EntityName);
        btRigidBody* rB = e.m_pRigidBody.get();

        btTransform tr= rB->getCenterOfMassTransform();
        btQuaternion quat;
        cout<<"try to set roll "<<roll<<" pitch "<<pitch<<" yaw "<<yaw<<endl;
        quat.setEulerZYX(yaw,pitch,roll);
        tr.setRotation(quat);

        rB->setCenterOfMassTransform(tr);
        PrintEntityRotation(EntityName);
    }



    // --------------------------------------------------------------------------------------------------------------
    void GetEntityRotation(string EntityName, double& roll, double& pitch, double& yaw)
    {
        Entity e = m_pPhys->getEntity(EntityName);
        btRigidBody* rB = e.m_pRigidBody.get();
        btQuaternion quat = rB->getCenterOfMassTransform().getRotation();
        roll = quat.getX();
        pitch = quat.getY();
        yaw = quat.getZ();
    }


    // --------------------------------------------------------------------------------------------------------------
    void PrintEntityRotation(string EntityName)
    {
        Entity e = m_pPhys->getEntity(EntityName);
        btRigidBody* rB = e.m_pRigidBody.get();
        btQuaternion quat = rB->getCenterOfMassTransform().getRotation();
        double roll = quat.getX();
        double pitch = quat.getY();
        double yaw = quat.getZ();
        cout<<"get roll "<<roll<<" pitch "<<pitch<<" yaw "<<yaw<<endl;
    }




    /// force, steering and Torque
    // --------------------------------------------------------------------------------------------------------------
    void SetFriction(string name, double F)
    {
        Entity e = m_pPhys->getEntity(name);
        btRigidBody* rB = e.m_pRigidBody.get(); //.m_pRigidBody->

        rB->setFriction(F);
    }



    // --------------------------------------------------------------------------------------------------------------
    void ApplyForceToEntity(string name, double F)
    {
      Entity e = m_pPhys->getEntity(name);
      btRigidBody* rB = e.m_pRigidBody.get(); //.m_pRigidBody->

      btVector3 force;
      force.setX(F);
      force.setY(0);
      force.setZ(0);

      rB->applyCentralForce(force);
    }



    // --------------------------------------------------------------------------------------------------------------
    void ApplyTorque(string sBodyFullName, Eigen::Vector3d eTorque)
    {
        Entity e = m_pPhys->getEntity(sBodyFullName);
        btRigidBody* rB = e.m_pRigidBody.get();

        btVector3 Torque;

        Torque.setX(eTorque[0]);
        Torque.setY(eTorque[1]);
        Torque.setZ(eTorque[2]);

        Torque = (rB->getCenterOfMassTransform().getBasis())*Torque;

        rB->applyTorque(Torque);
    }





    // --------------------------------------------------------------------------------------------------------------
    void ApplySteering(string sBodyFullName, Eigen::Vector3d eSteering)
    {
        Entity Wheel = m_pPhys->getEntity(sBodyFullName);

        btRigidBody* pWheel = Wheel.m_pRigidBody.get();
        btVector3 eSteer;
        eSteer[0] = eSteering[0];
        eSteer[1] = eSteering[1];
        eSteer[2] = eSteering[2];

//        btTransform T;
//        T = pWheel->getCenterOfMassTransform();
//        btQuaternion quat;
//        quat.setRotation(btVector3(0, 0, 1), T.getRotation().getAngle() + eSteer[2]);
//        T.setRotation(quat);

//        pWheel->setCenterOfMassTransform(T);
//        pWheel->
        pWheel->applyTorque(eSteer);
    }


    /// serialize and deserialize. experimental .. not working now
    // --------------------------------------------------------------------------------------------------------------
    // return the serialize char* buffer for rigid body with 'bodyname'
    void SerializeRigidBodyToChar(string sBodyName, const unsigned char*& pData, int& iDataSize)
    {
        Entity e = m_pPhys->getEntity(sBodyName);
        btRigidBody* rB = e.m_pRigidBody.get();

        int maxSerializeBufferSize = 1024*1024*5;
        btDefaultSerializer serializer (maxSerializeBufferSize);

        serializer.startSerialization();
        serializer.registerNameForPointer(rB,"fuck!!!!");
        rB->serializeSingleObject(&serializer);
        serializer.finishSerialization();

        pData = serializer.getBufferPointer();
        iDataSize = serializer.getCurrentBufferSize();
    }


    // --------------------------------------------------------------------------------------------------------------
    // get world transform from serialize data
//    bool ApplySerializeInforToAllBelongBody(string sBodyName, const unsigned char* pData, int iDataSize)
//    {
//        btBulletWorldImporter* fileLoader = new btBulletWorldImporter(m_pPhys->GetDynamicsWorld());
//        fileLoader->setVerboseMode(true);
//        fileLoader->loadFileFromMemory((char*)pData, iDataSize);
//        cout<<"num of rigid body "<< fileLoader->getNumRigidBodies()<<". num of collection shape "<<fileLoader->getNumCollisionShapes()<<". Num of fix body"<<fileLoader->getNumTriangleInfoMaps()<<endl;

//        return true;
//    }

    // add more function here...



private:
    Phys*                                                  m_pPhys;

};


#endif // BULLETMODELGRAPHAGENT_H
