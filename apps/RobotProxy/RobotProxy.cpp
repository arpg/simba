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

#include <URDFParser/RobotProxyURDFParser.h>            // for parse URDF file
#include <World/WorldManager.h>               // for manage world information
#include <Network/NetworkManager.h>           // for manager Network
#include <Device/SimDeviceManager.h>          // for manage all Simdevice of the Robot we control
#include <Robots/RobotsManager.h>             // for manage all robots
#include <Robots/SimRobot.h>                  // for user's robot
#include <ModelGraph/PhyModelGraphAgent.h>    // for communicate between Physic Engine and Model Graph

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
"      --Statekeeper Option -s      input 'StateKeeperName' or 'WithoutStateKeeper' or 'WithoutNetwork'"


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
        SimRobot*                   m_pMainRobot;            // user's robot. will be delete in final version of robot proxy (as we are not going to key control main robot in proxy)
        WorldManager                m_WorldManager;
        SimDeviceManager            m_SimDeviceManager;
        RobotsManager               m_RobotManager;
        NetworkManager              m_NetworkManager;
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

        // 1, parse world.xml file.
            if( ParseWorld(m_sWorldURDFFile.c_str(), m_WorldManager) != true)
            {
                cout<<"[RobotProxy] Cannot parse "<< m_sWorldURDFFile<<". Exit."<<endl;
                exit(-1);
            }



        // 2, Read Robot.xml file. Get reference to xmldocument.
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
            m_RobotManager.Init(m_sProxyName, sServerName ,m_PhyMGAgent,m_Render);
            if( m_RobotManager.AddRobot(RobotURDF, m_sProxyName) !=true)
            {
                cout<<"[RobotProxy] Cannot add new robot to RobotManager."<<endl;
                exit(-1);
            }

            // get pointer of main robot. (*** temporty code. need to be delete once we implement node controller ***)
            m_pMainRobot = m_RobotManager.GetMainRobot();



        // 5, init NetWork
            if(m_NetworkManager.initNetwork(m_sProxyName,  &m_SimDeviceManager, &m_RobotManager,  sServerName)!=true)
            {
                cout<<"[RobotProxy] You choice to connect to '"<<sServerName<<"' but we cannot init Nextwrok. Please make sure the StateKeeper is running."<<endl;
                exit(-1);
            }



        // 6, init Sim Device (SimCam, SimGPS, SimVicon, etc)
            if(m_SimDeviceManager.Init(m_PhyMGAgent,  m_rSceneGraph, RobotURDF, m_sProxyName)!= true)
            {
                cout<<"[RobotProxy] Cannot init SimDeviceManager."<<endl;
                exit(-1);
            }



        // 7, if run in with network mode, proxy network will publish sim device
            if(sServerName !="WithoutNetwork")
            {
                if(m_NetworkManager.initDevices()!=true)
                {
                    cout<<"[RobotProxy] Cannot init Nextwrok"<<endl;
                    exit(-1);
                }
            }
        }

        //----------------------------------------------------------------------------------------------------------------------------------------------
        // Re-allocate the simulator each time. ***** Need to be init from URDF later *****
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
            m_ground.SetExtent( 200,200, dThickness );
            m_rSceneGraph.AddChild( &m_ground );

            BoxShape bs = BoxShape(100, 100, 1/2.0f);
            Body* ground = new Body("Ground", bs);
            ground->m_dMass = 0;
            ground->SetWPose( m_ground.GetPose4x4_po() );
            m_PhyMGAgent.m_Agent.GetPhys()->RegisterObject(ground, "Ground", m_ground.GetPose());

            // maybe dangerous to always reload meshes?  maybe we should separate Init from Reset?
            try
            {
//                cout<<"try init mesh: "<<m_WorldManager.m_sMesh<<endl;
//                m_Map.Init(m_WorldManager.m_sMesh);
//                m_Map.SetPerceptable(true);
//                m_Map.SetScale(m_WorldManager.iScale);
//                m_Map.SetPosition(m_WorldManager.vWorldPose[0], m_WorldManager.vWorldPose[1],m_WorldManager.vWorldPose[2]);
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
            Eigen::Vector6d  dCommand;
            vector<string> vBodyFullName;
            string sMainRobotName = m_pMainRobot->GetRobotName();

            dCommand<<0, 0, 0, 0, 0, -0.01;
            vBodyFullName.push_back("FRWheel@"+sMainRobotName);
            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);

            dCommand<<0, 0, 0, 0, 0, -0.01;
            vBodyFullName.push_back("FLWheel@"+sMainRobotName);
            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);

            m_SimDeviceManager.GetSimpleController("MController")->ApplyCommand();
        }

        ///////////////////////////////////////////////////////////////////
        void RightKey()
        {
            Eigen::Vector6d  dCommand;
            vector<string> vBodyFullName;
            string sMainRobotName = m_pMainRobot->GetRobotName();

            dCommand<<0, 0, 0, 0, 0, 0.01;
            vBodyFullName.push_back("FRWheel@"+sMainRobotName);
            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);

            dCommand<<0, 0, 0, 0, 0, 0.01;
            vBodyFullName.push_back("FLWheel@"+sMainRobotName);
            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);

            m_SimDeviceManager.GetSimpleController("MController")->ApplyCommand();
        }

        ///////////////////////////////////////////////////////////////////
        void ForwardKey()
        {
            Eigen::Vector6d  dCommand;
            vector<string> vBodyFullName;
            string sMainRobotName = m_pMainRobot->GetRobotName();

            dCommand<<0, -8000, 0,0,0,0;
            vBodyFullName.push_back("BRWheel@"+sMainRobotName);
            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);

            dCommand<<0, 8000, 0,0,0,0;
            vBodyFullName.push_back("BLWheel@"+sMainRobotName);
            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);

            m_SimDeviceManager.GetSimpleController("MController")->ApplyCommand();

        }

        ///////////////////////////////////////////////////////////////////
        void ReverseKey()
        {
            Eigen::Vector6d  dCommand;
            vector<string> vBodyFullName;
            string sMainRobotName = m_pMainRobot->GetRobotName();

            dCommand<<0, 8000, 0,0,0,0;
            vBodyFullName.push_back("BRWheel@"+sMainRobotName);
            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);
//            m_SimDeviceManager.GetSimpleController("MController")->ApplyCommand();

            dCommand<<0, -8000, 0,0,0,0;
            vBodyFullName.push_back("BLWheel@"+sMainRobotName);
            m_SimDeviceManager.GetSimpleController("MController")->UpdateCommand(vBodyFullName,dCommand);

            m_SimDeviceManager.GetSimpleController("MController")->ApplyCommand();
        }

        // ---- roll
        void IncreaseCamRoll()
        {
//            double roll,pitch,yaw;
//            roll = 0.1;
//            pitch = 0;
//            yaw = 0;
//            m_SimRobot->m_Controller.SetRotation("RCameraRGB",roll,pitch,yaw);
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

        // ---- Step Forward
        void StepForward( void )
        {
            m_PhyMGAgent.m_Agent.GetPhys()->StepSimulation();
            m_Render.UpdateScene();
        }


        //-------------------------------------------------------------------------------------------------------------------------------------
        // scan all sim cam and set image to pangolin window. Now only support up to two window.
        bool SetImagesToWindow(SceneGraph::ImageView& LSimCamWnd, SceneGraph::ImageView& RSimCamWnd )
        {
            int WndCounter = 0;

            for(unsigned int i =0 ; i!= m_SimDeviceManager.m_SimDevices.size(); i++)
            {
                SimDeviceInfo Device = m_SimDeviceManager.m_SimDevices[i];

                for(unsigned int j=0;j!=Device.m_vSensorList.size();j++){

                    string sSimCamName = Device.m_vSensorList[j];
                    SimCam* pSimCam = m_SimDeviceManager.GetSimCam(sSimCamName);

                    SceneGraph::ImageView* ImageWnd;

                    // get pointer to window
                    if(WndCounter == 0)
                    {
                        ImageWnd = &LSimCamWnd;
                    }
                    else if(WndCounter == 1)
                    {
                        ImageWnd = &RSimCamWnd;
                    }

                    WndCounter++;

                    // set image to window
                            if (pSimCam->m_iCamType == 5)       // for depth image
                            {
                                float* pImgbuf = (float*) malloc( pSimCam->g_nImgWidth * pSimCam->g_nImgHeight * sizeof(float) );

                                if(pSimCam->capture(pImgbuf)==true)
                                {
                                    ImageWnd->SetImage(pImgbuf, pSimCam->g_nImgWidth, pSimCam->g_nImgHeight,  GL_INTENSITY, GL_LUMINANCE, GL_FLOAT);
                                    free(pImgbuf);
                                }
                                else
                                {
                                    cout<<"[SetImagesToWindow] Set depth Image fail"<<endl;
                                    return false;
                                }
                            }
                            else if(pSimCam->m_iCamType == 2)   // for RGB image
                            {
                                char* pImgbuf= (char*)malloc (pSimCam->g_nImgWidth * pSimCam->g_nImgHeight * 3);

                                if(pSimCam->capture(pImgbuf)==true)
                                {
                                    ImageWnd->SetImage(pImgbuf, pSimCam->g_nImgWidth, pSimCam->g_nImgHeight, GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE);
                                    free(pImgbuf);
                                }
                                else
                                {
                                    cout<<"[SetImagesToWindow] Set RGB Image fail"<<endl;
                                    return false;
                                }
                            }
                            else if(pSimCam->m_iCamType == 1)   // for show gray scale image
                            {
                                char* pImgbuf= (char*)malloc (pSimCam->g_nImgWidth * pSimCam->g_nImgHeight);
                                if(pSimCam->capture(pImgbuf)==true)
                                {
                                    ImageWnd->SetImage(pImgbuf, pSimCam->g_nImgWidth, pSimCam->g_nImgHeight,  GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE);
                                    free(pImgbuf);
                                }
                                else
                                {
                                    cout<<"[SetImagesToWindow] Set Gray Image fail"<<endl;
                                    return false;
                                }
                            }
                    }
            }

            return true;
        }

//        void TestSerialize()
//        {
//            unsigned const char* pData;
//            int     iDataSize;
//            for(int i=0;i!=m_RobotManager.GetMainRobot()->GetAllBodyName().size();i++)
//            {
//                string sName = m_RobotManager.GetMainRobot()->GetAllBodyName()[i];
////                cout<<"try to serialize "<<sName<<endl;
//                m_RobotManager.m_PhyMGAgent.m_Agent.SerializeRigidBodyToChar(sName,pData,iDataSize);
////                m_RobotManager.m_PhyMGAgent.m_Agent.SerializeDynmaticWorldToChar(pData,iDataSize);
//                m_PhyMGAgent.m_Agent.ApplySerializeInforToAllBelongBody(sName,pData,iDataSize);
//            }
//        }

};



///////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    // parse command line arguments
    GetPot cl( argc, argv );
    std::string sProxyName = cl.follow( "SimWorld", 1,"-n" );
    std::string sRobotURDF = cl.follow("/Users/malu/Code/Luma/Sim/urdf/Robot.xml",1,"-r");
    std::string sWorldURDF = cl.follow( "/Users/malu/Code/Luma/Sim/urdf/World.xml", 1, "-w" );
    std::string sServerOption = cl.follow("WithoutStateKeeper", 1 ,"-s");


    if( argc != 9){
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
    const double far = 2*size;
    const double near = far / 1E3;
    cout<<"center is "<<center<<endl;

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

    // window for display image capture from simcam
    SceneGraph::ImageView LSimCamImage(true,true);
    LSimCamImage.SetBounds( 0.0, 0.5, 0.8, 1.0, 512.0f/384.0f );

    // window for display image capture from simcam
    SceneGraph::ImageView RSimCamImage(true,true);
    RSimCamImage.SetBounds( 0.5, 1.0, 0.8, 1.0, 512.0f/384.0f );


    // Add our views as children to the base container.
    pangolin::DisplayBase().AddDisplay( view3d );
    pangolin::DisplayBase().AddDisplay( LSimCamImage );
    pangolin::DisplayBase().AddDisplay( RSimCamImage );


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

//    while(mProxy.m_ProxyNetwork.m_SubscribeNum == 0)
//    {
//        cout<<"["<<sProxyName<<"] wait for RPG Device to register"<<endl;
//        sleep(1);
//    }


    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( !pangolin::ShouldQuit() )
    {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        view3d.Activate(stacks3d);

        // 1. Update physic and scene
        mProxy.m_PhyMGAgent.m_Agent.GetPhys()->DebugDrawWorld();
        mProxy.m_PhyMGAgent.m_Agent.GetPhys()->StepSimulation();
        mProxy.m_Render.UpdateScene();

        // 2. update all sim device
        mProxy.m_SimDeviceManager.UpdateAlLDevice();

        // 3. update network
        mProxy.m_NetworkManager.UpdateNetWork();

        // 4. show image in current window
        mProxy.SetImagesToWindow(LSimCamImage,RSimCamImage);

        // 5. reflash screen
        pangolin::FinishGlutFrame();
    }

    return 0;
}

