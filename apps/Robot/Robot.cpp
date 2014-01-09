/*

   Simple example of robot Sense, Plan, Act loop.

 */

#include <iostream>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>    // for reference counting pointers
#include <Eigen/Eigen>             // for vector maths
#include <pangolin/pangolin.h>     // for open GL state management
#include <SceneGraph/SceneGraph.h> // for open GL scene graph

#include <Utils/GetPot>                  // for command line parsing
#include <Utils/CVarHelpers.h>           // for parsing Eigen vars as CVars
#include <CVars/CVar.h>            // for glconsole

//#include <RPG/Utils/InitCam.h>  Deprecated
#include <HAL/Camera/CameraDevice.h>
#include <HAL/Camera/CameraDriverInterface.h>


using namespace std;
using namespace CVarUtils;
using namespace pangolin;
using namespace SceneGraph;
using namespace boost;

#define USAGE \
"USAGE: Robot -n <RobotName> -n <RobotProxyName>\n"\
"      --URDFFileName, -n           Name of URDF for this Robot.\n"\

class Robot
{
    public:
        ///////////////////////////////////////////////////////////////////
        SceneGraph::GLGrid          m_grid;
        SceneGraph::GLSceneGraph&   m_rSceneGraph;
        std::string                 m_sRobotName;
        std::string                 m_sRobotProxyName;

        Eigen::Vector6d             m_command;
        hal::Camera                 m_Cam;

        ///////////////////////////////////////////////////////////////////
        Robot(
                SceneGraph::GLSceneGraph& glGraph,  //< Input: reference to glGraph
                const char* URDFFileName
                )
            : m_rSceneGraph( glGraph )
        {

            // init command
            m_command<<1,
                    1,
                    1,
                    1,
                    1,
                    1;


            if(  hal::Camera(m_Cam, "RCamera", URDFFileName) == false)
            {
                cout<<"[RobotCode] cannot init camera"<<endl;
                exit(-1);
            }
        }

        ///////////////////////////////////////////////////////////////////
        void LeftKey()
        {
            m_command(5,0)=m_command(5,0)-0.1;
        }

        ///////////////////////////////////////////////////////////////////
        void RightKey()
        {
            m_command(5,0)=m_command(5,0)+0.1;
        }

        ///////////////////////////////////////////////////////////////////
        void ForwardKey()
        {
            m_command(0,0)=m_command(0,0)+10.0;
        }

        ///////////////////////////////////////////////////////////////////
        void ReverseKey()
        {
            m_command(0,0)=m_command(0,0)-10.0;
        }




//        // This function will send command to robot proxy. The command is linear and angular Force.
//        bool PublishCommandToRobotProxy()
//        {
//            boost::mutex::scoped_lock lock(m_Mutex); // careful

//            m_iTimestep++;

//            CommandMsg mCommand;
//            mCommand.set_linear_force(m_command(0,0));
//            mCommand.set_angular_force(m_command(5,0));
//            mCommand.set_time_step(m_iTimestep);

//            int iMaxTry=10;
//            bool bStatus=false;
//            while (bStatus==false)
//            {
//               bStatus=m_Node.publish( "Command", mCommand);
//               if(bStatus==true)
//               {
//                   // reset command after we send it
//                   m_command<<0,
//                              0,
//                              0,
//                              0,
//                              0,
//                              0;
//                   cout<< "[Publish] Publish " <<m_sRobotName<<" Command to RobotProxy success. Timestep  is: "<<m_iTimestep<<endl;
//                   return true;
//               }
//               else if(bStatus==false && iMaxTry!=0)
//               {
//                   cout<< "[Publish] ERROR: publishing command."<<endl;
//                   iMaxTry--;
//                   usleep(10000);
//               }
//               else
//               {
//                   return false;
//               }
//            }
//        }
};



///////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    // parse command line arguments
    const char* URDFFileName = argv[1];



    // Create OpenGL window in single line thanks to GLUT
    pangolin::CreateGlutWindowAndBind("Robot Main",640,480);
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
    glewInit();

    // sinle application context holds everything
    SceneGraph::GLSceneGraph  glGraph;
    Robot app( glGraph, URDFFileName); // initialize exactly one RobotProxy

    pangolin::View& container = pangolin::DisplayBase();

    // We define a special type of view which will accept image data
    // to display and set its bounds on screen.
    ImageView viewImage(true,false);
    viewImage.SetBounds(0.0, 1.0, 0, 1.0, (double)640/480);
    container.AddDisplay(viewImage);

/// Key
    // register a keyboard hook to trigger the reset method

    // simple asdw control
    RegisterKeyPressCallback( 'a', bind( &Robot::LeftKey, &app ) );
    RegisterKeyPressCallback( 'A', bind( &Robot::LeftKey, &app ) );

    RegisterKeyPressCallback( 's', bind( &Robot::ReverseKey, &app ) );
    RegisterKeyPressCallback( 'S', bind( &Robot::ReverseKey, &app ) );

    RegisterKeyPressCallback( 'd', bind( &Robot::RightKey, &app ) );
    RegisterKeyPressCallback( 'D', bind( &Robot::RightKey, &app ) );

    RegisterKeyPressCallback( 'w', bind( &Robot::ForwardKey, &app ) );
    RegisterKeyPressCallback( 'W', bind( &Robot::ForwardKey, &app ) );

    // </Pangolin boilerplate>

    std::vector< rpg::ImageWrapper > vImages; // container for images

    while( !pangolin::ShouldQuit() )
    {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // ACT: Send commands to the robot
//      if (app.PublishCommandToRobotProxy()==true)
//      {

        if( !app.m_Cam.Capture(vImages) )
        {
           std::cout << "Error getting images." << std::endl;
        }

        viewImage.SetImage(vImages[0].Image.data, vImages[0].width(), vImages[0].height(), GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE); //show RGB image
        viewImage.SetImage(vImages[1].Image.data, vImages[1].width(), vImages[1].height(), GL_INTENSITY, GL_LUMINANCE, GL_FLOAT); //show depth image

        usleep(1E6/30);

        pangolin::FinishGlutFrame();
    }

    return 0;
}

