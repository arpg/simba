/*
   RobotProxy, By luma. 2013.03
 */ 

#include <iostream>
#include <boost/bind.hpp>         
#include <boost/shared_ptr.hpp>               // for reference counting pointers
#include <Eigen/Eigen>                        // for vector maths
#include <pangolin/pangolin.h>                // for open GL state management
#include <SceneGraph/SceneGraph.h>            // for open GL scene graph
#include <Utils/GetPot>                       // for command line parsing
#include <Utils/CVarHelpers.h>                // for parsing Eigen vars as CVars
#include <CVars/CVar.h>                       // for glconsole

#include <URDFParser/URDFParser.h>            // for parse URDF file
#include <World/WorldManager.h>               // for manage world information
#include <Robots/RobotsManager.h>             // for manage all robots
#include <Robots/SimRobot.h>                  // for user's robot
#include <ModelGraph/PhyModelGraphAgent.h>    // for communicate between Physic Engine and ModelNode Graph

using namespace std;
using namespace CVarUtils;
using namespace pangolin;
using namespace SceneGraph;
using namespace boost;


#define USAGE    \
"USAGE: Robot -n <ProxyName> -r <robot.xml directory> -w <world.xml directory> -s <StateKeeper Option>\n"\
"      Options:\n"\
"      --ProxyName, -n              Name of this RobotProxy.\n"\
"      --Robot.xml, -r              Directory where robot.xml is stored.\n"\
"      --World.xml, -w              Directory where world.xml is store.\n"\
"      --Statekeeper Option -s      either 'StateKeeperName' or 'WithoutServer'"


class RobotProxy
{
    public:
        ///////////////////////////////////////////////////////////////////
        std::string                 m_sProxyName;
        std::string                 m_sRobotURDFFile;
        std::string                 m_sWorldURDFFile;

        SceneGraph::GLLight         m_light;
        SceneGraph::GLBox           m_ground;
        SceneGraph::GLGrid          m_grid;
        SceneGraph::GLSceneGraph&   m_rSceneGraph;
        SceneGraph::GLMesh          m_Map;                 // mesh for the world.

        Render                      m_Render;
        SimRobot*                   m_SimRobot;            // user's robot.
        WorldManager                m_WorldManager;
        RobotsManager               m_RobotManager;
        PhyModelGraphAgent          m_PhyMGAgent;          // for one sim proxy, there is one PhyAgent

        // TODO TODO
        ///////////////////////////////////////////////////////////////////
        RobotProxy(
                SceneGraph::GLSceneGraph& glGraph,  //< Input: reference to glGraph
                const std::string& sProxyName,      //< Input: name of robot proxy
                const std::string& sRobotURDF,        //< Input: location of meshes, models, maps etc
                const std::string& sWorldURDF,
                const std::string& sServerName
                )
            : m_rSceneGraph( glGraph )
        {
            m_sProxyName = sProxyName;
            m_sWorldURDFFile = sWorldURDF;
            m_sRobotURDFFile = sRobotURDF;

        // 1, parse world.xml file
            if( ParseWorld(m_sWorldURDFFile.c_str(), m_WorldManager) != true)
            {
                cout<<"[RobotProxy] Cannot parse "<< m_sWorldURDFFile<<". Exit."<<endl;
                exit(-1);
            }

        // 2, Read Robot,xml file. Get reference to xmldocument.
            XMLDocument RobotURDF;
            if(GetXMLdoc(sRobotURDF, RobotURDF)!= true)
            {
                cout<<"[RobotProxy] Cannot open "<<sRobotURDF<<endl;
                exit(-1);
            }

        // 3, Init Agent between Physic Engine and ModelGraph
            if(m_PhyMGAgent.init()!=true)
            {
                cout<<"[RobotProxy] Cannot init Physic ModelGraph Agent."<<endl;
                exit(-1);
            }

        // 4, Init user's Robot and add it to RobotManager
            m_RobotManager.Init(m_PhyMGAgent,m_Render);
            if( m_RobotManager.AddRobot(RobotURDF, m_sProxyName) !=true)
            {
                cout<<"[RobotProxy] Cannot add new robot to RobotManager."<<endl;
                exit(-1);
            }

            m_SimRobot = m_RobotManager.m_mSimRobotsList.begin()->second;

        }


        // Re-allocate the simulator each time
        void InitReset()
        {
            m_rSceneGraph.Clear();

            m_light.SetPosition( m_WorldManager.vLightPose[0],  m_WorldManager.vLightPose[1],  m_WorldManager.vLightPose[2]);
            m_rSceneGraph.AddChild( &m_light );

            m_grid.SetNumLines(20);
            m_grid.SetLineSpacing(1);
            m_rSceneGraph.AddChild(&m_grid);

            double dThickness = 1;
            m_ground.SetPose( 0,0, dThickness/2.0,0,0,0 );
            m_ground.SetExtent( 2000,2000, dThickness );
            m_rSceneGraph.AddChild( &m_ground );

            BoxShape bs = BoxShape(2000, 2000, 1/2.0f);
            Body* ground = new Body("Ground", bs);
            ground->m_dMass = 0;
            ground->SetWPose( m_ground.GetPose4x4_po() );
            m_PhyMGAgent.m_Agent.GetPhys()->RegisterObject(ground, "Ground", m_ground.GetPose());

            // maybe dangerous to always reload meshes?  maybe we should separate Init from Reset?
            try
            {
//                cout<<"try init mesh: "<<m_URDFBuildWorld.m_sMesh<<endl;
//                m_Map.Init(m_URDFBuildWorld.m_sMesh);
//                m_Map.SetPerceptable(true);
//                m_Map.SetScale(m_URDFBuildWorld.iScale);
//                m_Map.SetPosition(m_URDFBuildWorld.vWorldPose[0], m_URDFBuildWorld.vWorldPose[1],m_URDFBuildWorld.vWorldPose[2]);
//                m_rSceneGraph.AddChild( &m_Map );
            } catch (std::exception e) {
                printf( "Cannot load world map\n");
                exit(-1);
            }

            m_Render.AddToScene( &m_rSceneGraph );
        }



        //----------------------------------------------------------------------------------------------------------------------------------------------
        void LeftKey()
        {
            m_SimRobot->m_Controller.ApplySteering("FLWheel",0.01);
            m_SimRobot->m_Controller.ApplySteering("BLWheel",0.01);
        }

        ///////////////////////////////////////////////////////////////////
        void RightKey()
        {
            m_SimRobot->m_Controller.ApplySteering("FLWheel",-0.01);
            m_SimRobot->m_Controller.ApplySteering("BLWheel",-0.01);
        }

        ///////////////////////////////////////////////////////////////////
        void ForwardKey()
        {
            m_SimRobot->m_Controller.ApplyTorque("BRWheel",80000);
            m_SimRobot->m_rPhyMGAgent.m_Agent.GetPhys()->PrintAllEntityName();
            m_SimRobot->m_Controller.ApplyTorque("BLWheel",80000);
        }

        ///////////////////////////////////////////////////////////////////
        void ReverseKey()
        {
            m_SimRobot->m_Controller.ApplyTorque("FRWheel",-80000);
            m_SimRobot->m_Controller.ApplyTorque("FLWheel",-80000);
        }

        // ---- roll
        void IncreaseCamRoll()
        {
          double roll,pitch,yaw;
            roll = 0.1;
            pitch = 0;
            yaw = 0;
            m_SimRobot->m_Controller.SetRotation("RCameraRGB",roll,pitch,yaw);
        }

        void DecreaseCamRoll()
        {
           double roll,pitch,yaw;
            roll = -0.1;
            pitch = 0;
            yaw = 0;
            m_SimRobot->m_Controller.SetRotation("RCameraRGB",roll,pitch,yaw);
        }

        // ---- pitch
        void IncreaseCamPitch()
        {
           double roll,pitch,yaw;
            roll = 0;
            pitch = 0.1;
            yaw = 0;
            m_SimRobot->m_Controller.SetRotation("RCameraRGB",roll,pitch,yaw);
        }

        void DecreaseCamPitch()
        {
            double roll,pitch,yaw;
            roll = 0;
            pitch = -0.1;
            yaw = 0;
            m_SimRobot->m_Controller.SetRotation("RCameraRGB",roll,pitch,yaw);
        }

        // ---- Yaw
        void IncreaseCamYaw()
        {
            double roll,pitch,yaw;
            roll = 0;
            pitch = 0;
            yaw = 0.1;
            m_SimRobot->m_Controller.SetRotation("RCameraRGB",roll,pitch,yaw);
        }

        void DecreaseCamYaw()
        {
            double roll,pitch,yaw;
            roll = 0;
            pitch = 0;
            yaw = -0.1;
            m_SimRobot->m_Controller.SetRotation("RCameraRGB",roll,pitch,yaw);
        }

        // ---- Step Forward
        void StepForward( void )
        {
            m_PhyMGAgent.m_Agent.GetPhys()->StepSimulation();
            m_Render.UpdateScene();
        }

};



///////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    // parse command line arguments
    GetPot cl( argc, argv );
    std::string sProxyName = cl.follow( "SimWorld", 1,"-n" );
    std::string sRobotURDF = cl.follow("/Users/malu/Code/Luma/Sim/urdf/Robot.xml",1,"-r");
    std::string sWorldURDF = cl.follow( "/Users/malu/Code/Luma/Sim/urdf/World.xml", 1, "-w" );
    std::string sServerOption = cl.follow("WithoutServer", 1 ,"-s");


    if( argc != 9 && argc!=0 ){
        puts(USAGE);
        return -1;
    }

    // Create OpenGL window in single line thanks to GLUT
    pangolin::CreateGlutWindowAndBind(sProxyName,640,480);
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
    glewInit();

    // sinle application context holds everything
    SceneGraph::GLSceneGraph  glGraph;
    RobotProxy mProxy( glGraph, sProxyName, sRobotURDF, sWorldURDF, sServerOption); // initialize exactly one RobotProxy
    mProxy.InitReset(); // this will populate the scene graph with objects and
    // register these objects with the simulator.


    //---------------------------------------------------------------------------------------------
    // <Pangolin boilerplate>
    const SceneGraph::AxisAlignedBoundingBox bbox = glGraph.ObjectAndChildrenBounds();
    const Eigen::Vector3d center = bbox.Center();
    const double size = bbox.Size().norm();
    const double far = 3*size;
    const double near = far / 1E3;

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState stacks3d(
            pangolin::ProjectionMatrix(640,480,420,420,320,240,near,far),
            pangolin::ModelViewLookAt(center(0), center(1) + size, center(2) - size/4,
                center(0), center(1), center(2), pangolin::AxisNegZ) );

    // We define a new view which will reside within the container.
    pangolin::View view3d;

    // We set the views location on screen and add a handler which will
    // let user input update the model_view matrix (stacks3d) and feed through
    // to our scenegraph
    view3d.SetBounds( 0.0, 1.0, 0.0, 0.8, -640.0f/480.0f );
    view3d.SetHandler( new SceneGraph::HandlerSceneGraph( glGraph, stacks3d) );
    view3d.SetDrawFunction( SceneGraph::ActivateDrawFunctor( glGraph, stacks3d) );

    // Add our views as children to the base container.
    pangolin::DisplayBase().AddDisplay( view3d );

    //---------------------------------------------------------------------------------------------
    // register a keyboard hook to trigger the reset method
    RegisterKeyPressCallback( pangolin::PANGO_CTRL + 'r', boost::bind( &RobotProxy::InitReset, &mProxy ) );

    // simple asdw control
    RegisterKeyPressCallback( 'a', bind( &RobotProxy::LeftKey, &mProxy ) );
    RegisterKeyPressCallback( 'A', bind( &RobotProxy::LeftKey, &mProxy ) );

    RegisterKeyPressCallback( 's', bind( &RobotProxy::ReverseKey, &mProxy ) );
    RegisterKeyPressCallback( 'S', bind( &RobotProxy::ReverseKey, &mProxy ) );

    RegisterKeyPressCallback( 'd', bind( &RobotProxy::RightKey, &mProxy ) );
    RegisterKeyPressCallback( 'D', bind( &RobotProxy::RightKey, &mProxy ) );

    RegisterKeyPressCallback( 'w', bind( &RobotProxy::ForwardKey, &mProxy ) );
    RegisterKeyPressCallback( 'W', bind( &RobotProxy::ForwardKey, &mProxy ) );
    RegisterKeyPressCallback( ' ', bind( &RobotProxy::StepForward, &mProxy ) );

    // SimCam control
    RegisterKeyPressCallback( '8', bind( &RobotProxy::IncreaseCamPitch, &mProxy ) ); // up
    RegisterKeyPressCallback( '5', bind( &RobotProxy::DecreaseCamPitch, &mProxy ) ); // down
    RegisterKeyPressCallback( '4', bind( &RobotProxy::IncreaseCamYaw, &mProxy ) );// letf
    RegisterKeyPressCallback( '6', bind( &RobotProxy::DecreaseCamYaw, &mProxy ) );// right
    RegisterKeyPressCallback( '1', bind( &RobotProxy::IncreaseCamRoll, &mProxy ) );
    RegisterKeyPressCallback( '3', bind( &RobotProxy::DecreaseCamRoll, &mProxy ) );


    //---------------------------------------------------------------------------------------------

    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( !pangolin::ShouldQuit() )
    {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        view3d.Activate(stacks3d);

        // 1. Update physic and scene
        mProxy.m_PhyMGAgent.m_Agent.GetPhys()->DebugDrawWorld();
        mProxy.m_PhyMGAgent.m_Agent.GetPhys()->StepSimulation();
        mProxy.m_Render.UpdateScene();

        usleep(1E0/30);

        // 5. reflash screen
        pangolin::FinishGlutFrame();
    }

    return 0;
}

