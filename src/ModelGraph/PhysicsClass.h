#ifndef PHYSICSCLASS_H
#define PHYSICSCLASS_H

#include <boost/shared_ptr.hpp>
#include <bullet/btBulletDynamicsCommon.h>
#include <ModelGraph/Models.h>
#include <ModelGraph/SE3.h>
#include <bullet/LinearMath/btIDebugDraw.h>
#include <math.h>

using namespace std;


inline Eigen::Matrix4d
    getInverseTransformation (const Eigen::Matrix4d &transformation)
  {
    Eigen::Matrix4d transformation_inverse;
    float tx = transformation (0, 3);
    float ty = transformation (1, 3);
    float tz = transformation (2, 3);

    transformation_inverse (0, 0) = transformation (0, 0);
    transformation_inverse (0, 1) = transformation (1, 0);
    transformation_inverse (0, 2) = transformation (2, 0);
    transformation_inverse (0, 3) = - (transformation (0, 0) * tx + transformation (0, 1) * ty + transformation (0, 2) * tz);


    transformation_inverse (1, 0) = transformation (0, 1);
    transformation_inverse (1, 1) = transformation (1, 1);
    transformation_inverse (1, 2) = transformation (2, 1);
    transformation_inverse (1, 3) = - (transformation (1, 0) * tx + transformation (1, 1) * ty + transformation (1, 2) * tz);

    transformation_inverse (2, 0) = transformation (0, 2);
    transformation_inverse (2, 1) = transformation (1, 2);
    transformation_inverse (2, 2) = transformation (2, 2);
    transformation_inverse (2, 3) = - (transformation (2, 0) * tx + transformation (2, 1) * ty + transformation (2, 2) * tz);

    transformation_inverse (3, 0) = 0;
    transformation_inverse (3, 1) = 0;
    transformation_inverse (3, 2) = 0;
    transformation_inverse (3, 3) = 1;
    return transformation_inverse;
}

inline Eigen::Matrix<double,4,4> toEigen(const btTransform& T)
{
    Eigen::Matrix<btScalar,4,4> eT;
    T.getOpenGLMatrix(eT.data());
    return eT.cast<double>();
}

inline btTransform toBullet(const Eigen::Matrix<double,4,4>& T)
{
    btTransform bT;
    Eigen::Matrix<btScalar,4,4> eT = T.cast<btScalar>();
    bT.setFromOpenGLMatrix(eT.data());
    return bT;
}

inline btVector3 toBulletVec3(const Eigen::Vector3d& v)
{
    btVector3 bv;
    bv.setX(v(0));
    bv.setY(v(1));
    bv.setZ(v(2));
    return bv;
}

inline btVector3 toBulletVec3(const double x, const double y, const double z)
{
    btVector3 bv;
    bv.setX(x);
    bv.setY(y);
    bv.setZ(z);
    return bv;
}




class NodeMotionState : public btMotionState {
    public:
        NodeMotionState(ModelNode& obj, Eigen::Vector6d& wp)
            : object(obj)
        {
            m_WorldPose = _Cart2T(wp);
        }

        NodeMotionState(ModelNode& obj, Eigen::Matrix4d& wp)
            : object(obj), m_WorldPose(wp)
        {
        }

        virtual void getWorldTransform(btTransform &worldTrans) const {
            worldTrans = toBullet( m_WorldPose);
        }

        virtual void setWorldTransform(const btTransform &worldTrans) {            
            if (dynamic_cast<Body*>(&object))
            {
                Body* pBody = (Body*) &object;
                if (dynamic_cast<CylinderShape*>(pBody->m_CollisionShape))
                {
                    Eigen::Vector6d temp;
                    temp << 0, 0, 0, M_PI / 2, 0, 0;
                    Eigen::Matrix4d rot;
                    rot = toEigen(worldTrans);
                    rot = rot*_Cart2T(temp);
                    m_WorldPose = rot;
                    object.SetWPose(m_WorldPose);
                }
                else
                {
                    m_WorldPose = toEigen(worldTrans);
                    object.SetWPose(m_WorldPose);
                }
            }
            else
            {
                m_WorldPose = toEigen(worldTrans);
                object.SetWPose(m_WorldPose);
            }
        }


        ModelNode& object;
        Eigen::Matrix4d m_WorldPose;
};

// **** when delete a robot, make sure to delet all information in the following data struct *****
typedef  boost::shared_ptr<btCollisionShape>            CollisionShapePtr;
typedef  boost::shared_ptr<btRigidBody>                 RigidBodyPtr;
typedef  boost::shared_ptr<NodeMotionState>             NodeMotionStatePtr;




class DebugDraw : public btIDebugDraw
{
    int m_debugMode;

public:

     virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& fromColor, const btVector3& toColor)
    {
        glBegin(GL_LINES);
            glColor3f(fromColor.getX(), fromColor.getY(), fromColor.getZ());
            glVertex3d(from.getX(), from.getY(), from.getZ());
            glColor3f(toColor.getX(), toColor.getY(), toColor.getZ());
            glVertex3d(to.getX(), to.getY(), to.getZ());
        glEnd();
    }

    virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
    {
        drawLine(from,to,color,color);
    }

    virtual int getDebugMode( void ) const
    {
        return m_debugMode;
    }

    virtual void drawSphere (const btVector3& p, btScalar radius, const btVector3& color)
    {
        glColor4f (color.getX(), color.getY(), color.getZ(), btScalar(1.0f));
        glPushMatrix ();
        glTranslatef (p.getX(), p.getY(), p.getZ());

        int lats = 5;
        int longs = 5;

        int i, j;
        for(i = 0; i <= lats; i++) {
            btScalar lat0 = SIMD_PI * (-btScalar(0.5) + (btScalar) (i - 1) / lats);
            btScalar z0  = radius*sin(lat0);
            btScalar zr0 =  radius*cos(lat0);

            btScalar lat1 = SIMD_PI * (-btScalar(0.5) + (btScalar) i / lats);
            btScalar z1 = radius*sin(lat1);
            btScalar zr1 = radius*cos(lat1);

            glBegin(GL_QUAD_STRIP);
            for(j = 0; j <= longs; j++) {
                btScalar lng = 2 * SIMD_PI * (btScalar) (j - 1) / longs;
                btScalar x = cos(lng);
                btScalar y = sin(lng);

                glNormal3f(x * zr0, y * zr0, z0);
                glVertex3f(x * zr0, y * zr0, z0);
                glNormal3f(x * zr1, y * zr1, z1);
                glVertex3f(x * zr1, y * zr1, z1);
            }
            glEnd();
        }

        glPopMatrix();
    }

    virtual void drawBox (const btVector3& boxMin, const btVector3& boxMax, const btVector3& color, btScalar alpha)
    {
        btVector3 halfExtent = (boxMax - boxMin) * btScalar(0.5f);
        btVector3 center = (boxMax + boxMin) * btScalar(0.5f);
        //glEnable(GL_BLEND);     // Turn blending On
        //glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glColor4f (color.getX(), color.getY(), color.getZ(), alpha);
        glPushMatrix ();
        glTranslatef (center.getX(), center.getY(), center.getZ());
        glScaled(2*halfExtent[0], 2*halfExtent[1], 2*halfExtent[2]);
    //	glutSolidCube(1.0);
        glPopMatrix ();
        //glDisable(GL_BLEND);
    }

    virtual void	drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha)
    {
    //	if (m_debugMode > 0)
        {
            const btVector3	n=btCross(b-a,c-a).normalized();
            glBegin(GL_TRIANGLES);
            glColor4f(color.getX(), color.getY(), color.getZ(),alpha);
            glNormal3d(n.getX(),n.getY(),n.getZ());
            glVertex3d(a.getX(),a.getY(),a.getZ());
            glVertex3d(b.getX(),b.getY(),b.getZ());
            glVertex3d(c.getX(),c.getY(),c.getZ());
            glEnd();
        }
    }

    virtual void setDebugMode(int debugMode)
    {
        m_debugMode = debugMode;
//        std::cout<<"this is debug mode "<<debugMode<<std::endl;
    }

    virtual void	draw3dText(const btVector3& location,const char* textString)
    {
    }

    virtual void	reportErrorWarning(const char* warningString)
    {
        printf("%s\n",warningString);
    }

    virtual void	drawContactPoint(const btVector3& pointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)

    {
        btVector3 to=pointOnB+normalOnB*1;//distance;
        const btVector3&from = pointOnB;
        glColor4f(color.getX(), color.getY(), color.getZ(),1.f);
        //glColor4f(0,0,0,1.f);
        glBegin(GL_LINES);
        glVertex3d(from.getX(), from.getY(), from.getZ());
        glVertex3d(to.getX(), to.getY(), to.getZ());
        glEnd();


        //		glRasterPos3f(from.x(),  from.y(),  from.z());
        //		char buf[12];
        //		sprintf(buf," %d",lifeTime);
        //BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);


    }

};



class Entity
{
    public:
        Entity()
        {

        }

        ~Entity()
        {

        }

        Entity(
                string sName,
                CollisionShapePtr  pShape, //< Input:
                NodeMotionStatePtr  pMotionState, //< Input:
                RigidBodyPtr pRigidBody //< Input:
                )
        {
            m_sName        = sName;
            m_pShape       = pShape;
            m_pMotionState = pMotionState;
            m_pRigidBody   = pRigidBody;
        }


        const char* GetParentName()
        {
            if (m_pMotionState->object.m_pParent) {
                return m_pMotionState->object.m_pParent->GetName().c_str();
            }
            return NULL;
        }


//    private:
        string                  m_sName;
        CollisionShapePtr       m_pShape;
        NodeMotionStatePtr      m_pMotionState;
        RigidBodyPtr            m_pRigidBody;
};




class Phys
{
    public:

        RigidBodyPtr m_carBody;
        DebugDraw   m_DebugDrawer;
        ///////////////////////////////////////////////////////////////////
        Phys()
        {
            m_dTimeStep = 1.0/30.0;
            m_dGravity = 9.8;
            m_nMaxSubSteps = 10; // bullet -- for stepSimulation

        }

        void Init(
                double dGravity = 9.8,       //< Input:
                double dTimeStep = 1.0/60.0, //< Input:
                double nMaxSubSteps = 1     //< Input: for stepSimulation
                )
        {

            m_dTimeStep    = dTimeStep;
            m_dGravity     = dGravity;
            m_nMaxSubSteps = nMaxSubSteps;


            // Physics stuff see http://bulletphysics.org/mediawiki-1.5.8/index.php/Hello_World

            btCollisionDispatcher* ptr = new btCollisionDispatcher(&m_CollisionConfiguration);

            m_pDispatcher = boost::shared_ptr<btCollisionDispatcher>(ptr);

            // Build the broadphase (approximate collision detection)
            m_pBroadphase
                = boost::shared_ptr<btDbvtBroadphase>( new btDbvtBroadphase );
            m_pSolver
                = boost::shared_ptr<btSequentialImpulseConstraintSolver>( new btSequentialImpulseConstraintSolver );

            /// the main machine
            m_pDynamicsWorld = boost::shared_ptr<btDiscreteDynamicsWorld>(new btDiscreteDynamicsWorld(
                        m_pDispatcher.get(),
                        m_pBroadphase.get(),
                        m_pSolver.get(),
                        &m_CollisionConfiguration
                        ) );
            m_pDynamicsWorld->setGravity( btVector3(0,0,m_dGravity) );

            m_pDynamicsWorld->setDebugDrawer( &m_DebugDrawer );
            m_pDynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_FastWireframe + btIDebugDraw::DBG_DrawConstraints);

        }

        ///////////////////////////////////////////////////////////////////
        boost::shared_ptr<Entity> GetParentEntity( Entity &e )
        {
            if (e.GetParentName()) {
                return m_mEntities[e.GetParentName()];
            }
            return boost::shared_ptr<Entity>();
        }

        ///////////////////////////////////////////////////////////////////
        Eigen::Matrix4d GetRelativePose( Entity &rChild)
        {
            boost::shared_ptr<Entity> pParent = GetParentEntity(rChild);
            if (pParent) {
                Eigen::Matrix4d& Twp = pParent->m_pMotionState->m_WorldPose;
                Eigen::Matrix4d& Twc = rChild.m_pMotionState->m_WorldPose;
                return getInverseTransformation(Twp)*Twc; /// find Tinv in mvl or something
            }
            return Eigen::Matrix4d::Identity();
        }

        ///////////////////////////////////////////////////////////////////
        void RegisterObject(
                ModelNode *pItem,
                string sName,
                Eigen::Vector6d WorldPose,
                double dDefaultRestitution = 0
                )
        {
            btVector3 localInertia( 0, 0, 0 );
            double dMass = 0.0f;

            if (dynamic_cast<Body*>(pItem) != NULL) {
                Body* pNodeBody = (Body*) pItem;
                if (dynamic_cast<BoxShape*>( pNodeBody->m_CollisionShape ) != NULL) {
                    BoxShape* pCollisionShape = (BoxShape*) pNodeBody->m_CollisionShape;
//                    std::cout<<"Associating physics with a BOX!"<<std::endl;
                    dMass = pNodeBody->m_dMass;
                    btVector3 bounds = toBulletVec3( pCollisionShape->m_dDims );
                    CollisionShapePtr pBulletShape( new btBoxShape(bounds) );
                    btAssert((!pBulletShape || pBulletShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));
                    bool isDynamic = ( dMass != 0.f );
                    if( isDynamic ){
                        pBulletShape->calculateLocalInertia( dMass, localInertia );
                    }

                    NodeMotionStatePtr pMotionState( new NodeMotionState(*pNodeBody, WorldPose) );
                    btRigidBody::btRigidBodyConstructionInfo  cInfo( dMass, pMotionState.get(), pBulletShape.get(), localInertia );
                    RigidBodyPtr  pBulletBody( new btRigidBody(cInfo) );

                    double dDefaultContactProcessingThreshold = 0.001;
                    pBulletBody->setContactProcessingThreshold( dDefaultContactProcessingThreshold );
                    pBulletBody->setRestitution( dDefaultRestitution );

                    m_pDynamicsWorld->addRigidBody( pBulletBody.get() );

                    // save this object somewhere (to keep it's reference count above 0)
                    boost::shared_ptr<Entity> pEntity( new Entity );
                    pEntity->m_sName = sName;
                    pEntity->m_pRigidBody.swap(pBulletBody);
                    pEntity->m_pShape.swap(pBulletShape);
                    pEntity->m_pMotionState.swap(pMotionState);

                    int id = m_mEntities.size();
                    m_mEntities[sName] = pEntity;

                }
                else if (dynamic_cast<CylinderShape*>( pNodeBody->m_CollisionShape ) != NULL) {
                    CylinderShape* pCollisionShape = (CylinderShape*) pNodeBody->m_CollisionShape;
//                    std::cout<<"Associating physics with a CYLINDER!"<<std::endl;
                    dMass = pNodeBody->m_dMass;
                    btVector3 bounds = toBulletVec3( pCollisionShape->m_dRadius, 0.5*pCollisionShape->m_dThickness, pCollisionShape->m_dRadius );
                    CollisionShapePtr pBulletShape( new btCylinderShape(bounds) );
                    btAssert((!pBulletShape || pBulletShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));
                    bool isDynamic = ( dMass != 0.f );
                    if( isDynamic ){
                        pBulletShape->calculateLocalInertia( dMass, localInertia );
                    }

                    NodeMotionStatePtr pMotionState( new NodeMotionState(*pNodeBody, WorldPose) );
                    btRigidBody::btRigidBodyConstructionInfo  cInfo( dMass, pMotionState.get(), pBulletShape.get(), localInertia );
                    RigidBodyPtr  pBulletBody( new btRigidBody(cInfo) );

                    double dDefaultContactProcessingThreshold = 0.001;
                    pBulletBody->setContactProcessingThreshold( dDefaultContactProcessingThreshold );
                    pBulletBody->setRestitution( dDefaultRestitution );

                    m_pDynamicsWorld->addRigidBody( pBulletBody.get() );

                    // save this object somewhere (to keep it's reference count above 0)
                    boost::shared_ptr<Entity> pEntity( new Entity );
                    pEntity->m_sName = sName;
                    pEntity->m_pRigidBody.swap(pBulletBody);
                    pEntity->m_pShape.swap(pBulletShape);
                    pEntity->m_pMotionState.swap(pMotionState);

                    int id = m_mEntities.size();
                    m_mEntities[sName] = pEntity;
                }
            }
            else if (dynamic_cast<HingeJoint*>(pItem) != NULL) {
//                std::cout<<"Associating physics with a Hinge Joint!"<<std::endl;
                HingeJoint* pHJ = (HingeJoint*) pItem;

                boost::shared_ptr<Entity> pParent = m_mEntities[pHJ->m_pParentBody->GetName()];
                btRigidBody* pBodyA =  pParent->m_pRigidBody.get(); //localCreateRigidBody( 1.0f, tr, shape);

                boost::shared_ptr<Entity> pChild = m_mEntities[pHJ->m_pChildBody->GetName()];
                btRigidBody* pBodyB = pChild->m_pRigidBody.get();

                pBodyA->setActivationState(DISABLE_DEACTIVATION);
                pBodyB->setActivationState(DISABLE_DEACTIVATION);

                // add some data to build constraint frames
                Eigen::Vector6d v = pHJ->GetPose();
                Eigen::Vector3d v3;

                v3 = pHJ->m_dAxis1;
                btVector3 axisA = toBulletVec3( v3 );   // The axisA is the hinge axis direction in the A ref frame
                btVector3 axisB = toBulletVec3( v3 );   // The axisB is the hinge axis direction in the B ref frame

                if (dynamic_cast<CylinderShape*>(pHJ->m_pChildBody->m_CollisionShape)){
                    axisB = toBulletVec3(pHJ->m_dAxis2);
                }

                v3 = pHJ->m_dPivot1;
                btVector3 pivotA = toBulletVec3( v3 );  // The pivotA is the pivoting point in the A frame

                // Now convert the pivot axis in A to the pivot axis in B [pB = pA - vAB]
                Eigen::Vector6d vAB6 = pHJ->m_pChildBody->GetPose();
                v3[0] -= vAB6[0];
                v3[1] -= vAB6[1];
                v3[2] -= vAB6[2];

                btVector3 pivotB = toBulletVec3( v3 );  // The pivotB is the pivoting in the B frame

                btHingeConstraint* spHingeDynAB = new btHingeConstraint(*pBodyA, *pBodyB, pivotA, pivotB, axisA, axisB);

                double dLowerLimit = pHJ->m_dLowerLimit;
                double dUpperLimit = pHJ->m_dUpperLimit;
                spHingeDynAB->setLimit(dLowerLimit,dUpperLimit);


                // add constraint to world
                m_pDynamicsWorld->addConstraint(spHingeDynAB, true);

                // draw constraint frames and limits for debugging
//                spHingeDynAB->setDbgDrawSize(btScalar(5.f));

                // save pointer to hinge contraint
                string sHingeName = pHJ->GetName();
                m_mHingeJointList.insert(std::pair<std::string,btHingeConstraint*>(sHingeName,spHingeDynAB));

            }
            else if (dynamic_cast<Hinge2Joint*>(pItem) != NULL) {
//                std::cout<<"Associating physics with a Hinge2Joint!"<<std::endl;
                Hinge2Joint* pH2J = (Hinge2Joint*) pItem;

                boost::shared_ptr<Entity> pParent = m_mEntities[pH2J->m_pParentBody->GetName()];
                btRigidBody* pBodyA =  pParent->m_pRigidBody.get(); //localCreateRigidBody( 1.0f, tr, shape);


                boost::shared_ptr<Entity> pChild = m_mEntities[pH2J->m_pChildBody->GetName()];
                btRigidBody* pBodyB = pChild->m_pRigidBody.get();

                pBodyA->setActivationState(DISABLE_DEACTIVATION);
                pBodyB->setActivationState(DISABLE_DEACTIVATION);

                btVector3 axis1 = toBulletVec3( pH2J->m_dAxis1 );   // The axisA is the hinge axis direction in the A ref frame
                btVector3 axis2 = toBulletVec3( pH2J->m_dAxis2 );   // The axisB is the hinge axis direction in the B ref frame
                btVector3 anchor = toBulletVec3( pH2J->m_dAnchor ); // The anchor point in world coordinates

                btHinge2Constraint* spHingeDynAB = new btHinge2Constraint(*pBodyA, *pBodyB, anchor, axis1, axis2);
                spHingeDynAB->setLinearLowerLimit(toBulletVec3(pH2J->m_dLowerLimit));
                spHingeDynAB->setLinearUpperLimit(toBulletVec3(pH2J->m_dUpperLimit));
                spHingeDynAB->setAngularLowerLimit(toBulletVec3(pH2J->m_dLowerALimit));
                spHingeDynAB->setAngularUpperLimit(toBulletVec3(pH2J->m_dUpperALimit));
                spHingeDynAB->setDamping(0, pH2J->m_dDamping);

                // add constraint to world
                m_pDynamicsWorld->addConstraint(spHingeDynAB, true);

                // draw constraint frames and limits for debugging
                spHingeDynAB->setDbgDrawSize(btScalar(5.f));

                // save pointer to hinge2 contraint
                string sHingeName = pH2J->GetName();
                m_mHinge2JointList.insert(std::pair<std::string,btHinge2Constraint*>(sHingeName,spHingeDynAB));
            }

            return;
        }

        ///////////////////////////////////////////////////////////////////
        void DebugDrawWorld()
        {
            m_pDynamicsWorld->debugDrawWorld();
        }


        ///////////////////////////////////////////////////////////////////
        void StepSimulation()
        {
            m_pDynamicsWorld->stepSimulation( m_dTimeStep,  m_nMaxSubSteps );
            // go through entities and set the node relative poses

//            UpdateWheel();
        }

        ///------------------------------------------------------------------------------------------------------------------------ add by luma
        Entity getEntity(string name)
        {
//            PrintAllEntityName();
            if(m_mEntities.find(name) !=m_mEntities.end())
            {
                Entity e =*m_mEntities.find(name)->second;
                return e;
            }
            else
            {
                cout<<"Fatal Error! Cannot get entity '"<<name<<"'. Exit!"<<endl;
                Entity e =*m_mEntities.find(name)->second;
                return e;
            }
        }


        ///--- entity only include body ------------------------------------------------------------------------------------------- add by luma
        void PrintAllEntityName()
        {
            std::map<string, boost::shared_ptr<Entity> >::iterator iter = m_mEntities.begin();
            for(iter = m_mEntities.begin();iter!=m_mEntities.end(); iter++)
            {
                string sFullName = iter->first;
                cout<<"Get current Entity Full Name "<<sFullName<<endl;
            }
        }


        vector<string> GetAllEntityName()
        {
            vector<string> vNameList;

            std::map<string, boost::shared_ptr<Entity> >::iterator iter = m_mEntities.begin();
            for(iter = m_mEntities.begin();iter!=m_mEntities.end(); iter++)
            {
                string sFullName = iter->first;
                vNameList.push_back(sFullName);
            }

            return vNameList;
        }


        ///------------------------------------------------------------------------------------------------------------------------ add by luma
        btHinge2Constraint* getHinge2Constraint(string name)
        {
            std::map<string, btHinge2Constraint*>::iterator iter = m_mHinge2JointList.find(name);
            if(iter!=m_mHinge2JointList.end())
            {
                btHinge2Constraint* pHJ2 = m_mHinge2JointList.find(name)->second;
                return pHJ2;
            }
            else
            {
                cout<<"Fatal Error! Cannot get Hinge2joint "<<name<<endl;
                btHinge2Constraint* pHJ2 = m_mHinge2JointList.find(name)->second;
                return pHJ2;
            }
        }


        ///------------------------------------------------------------------------------------------------------------------------ add by luma
        btHingeConstraint* getHingeConstraint(string name)
        {
            std::map<string, btHingeConstraint*>::iterator iter = m_mHingeJointList.find(name);
            if(iter!=m_mHingeJointList.end())
            {
                btHingeConstraint* pHJ = iter->second;
                return pHJ;
            }
            else
            {
                cout<<"Fatal Error! Cannot get Hingejoint "<<name<<endl;
                btHingeConstraint* pHJ = iter->second;
                return pHJ;
            }
        }


        ///------------------------------------------------------------------------------------------------------------------------ add by luma
        void PrintAllConstraintName()
        {
            std::map<string, btHingeConstraint*>::iterator iter = m_mHingeJointList.begin();
            for(;iter!=m_mHingeJointList.end();iter++)
            {
                cout<<"get hinge constraint name "<<iter->first<<endl;
            }

            std::map<string, btHinge2Constraint*>::iterator iter1 = m_mHinge2JointList.begin();
            for(;iter1!=m_mHinge2JointList.end();iter1++)
            {
                cout<<"get hinge 2 constraint name "<<iter1->first<<endl;
            }
        }


        ///------------------------------------------------------------------------------------------------------------------------ add by luma
        void DeleteHingeConstraintFromDynamicsWorld(string sConstraintName)
        {
            btHingeConstraint* h = getHingeConstraint(sConstraintName);

            btDiscreteDynamicsWorld* dynmaicsWorld = m_pDynamicsWorld.get();

            dynmaicsWorld->removeConstraint(h);
        }


        ///------------------------------------------------------------------------------------------------------------------------ add by luma
        void DeleteHinge2ConstraintFromDynamicsWorld(string sConstraintName)
        {
            btHinge2Constraint* h = getHinge2Constraint(sConstraintName);

            btDiscreteDynamicsWorld* dynmaicsWorld = m_pDynamicsWorld.get();

            dynmaicsWorld->removeConstraint(h);

        }


        ///------------------------------------------------------------------------------------------------------------------------ add by luma
        // important! must remove constraint before remove rigid body
        void DeleteRigidbodyFromDynamicsWorld(string sBodyFullName)
        {
            Entity e = getEntity(sBodyFullName);
            btRigidBody* objectbody = e.m_pRigidBody.get();

            btDiscreteDynamicsWorld* dynmaicsWorld = m_pDynamicsWorld.get();

            for(int i = dynmaicsWorld->getNumCollisionObjects()-1;i>=0;i--)
            {
                btCollisionObject* obj =dynmaicsWorld->getCollisionObjectArray()[i];
                btRigidBody* body = btRigidBody::upcast(obj);

                if(body == objectbody)
                {
                    if (body && body->getMotionState())
                    {
                        delete body->getMotionState();
                    }
                    dynmaicsWorld->removeCollisionObject(obj);
                    delete obj;
                }
            }
        }


        btDynamicsWorld* GetDynamicsWorld()
        {
            btDynamicsWorld* pDynmaicsWorld= m_pDynamicsWorld.get();
            return pDynmaicsWorld;
        }


        ///------------------------------------------------------------------------------------------------------------------------ add by luma
        // remove all body and joint in m_mEntities map
        void EraseBodyInEntitiesList(string sEntityName)
        {
            boost::shared_ptr<Entity> pEntity =  m_mEntities.find(sEntityName)->second;
            pEntity.reset();
            cout<<"find entity "<<sEntityName<< " success"<<endl;

        }

        std::map<string, boost::shared_ptr<Entity> >           m_mEntities;        // map of all rigid bodys
        std::map<string, btHingeConstraint*>                   m_mHingeJointList;  // map of all hinge constraints
        std::map<string, btHinge2Constraint*>                  m_mHinge2JointList; // map of all hinge 2 constraints

        boost::shared_ptr<btDiscreteDynamicsWorld>             m_pDynamicsWorld;

    private:

        ///////////////////////////////////////////////////////////////////
        btDefaultCollisionConfiguration                        m_CollisionConfiguration;
        boost::shared_ptr<btCollisionDispatcher>               m_pDispatcher;
        boost::shared_ptr<btDbvtBroadphase>                    m_pBroadphase;
        boost::shared_ptr<btSequentialImpulseConstraintSolver> m_pSolver;
        double                                                 m_dTimeStep;
        double                                                 m_dGravity;
        int                                                    m_nMaxSubSteps;
};


#endif // PHYSICSCLASS_H
