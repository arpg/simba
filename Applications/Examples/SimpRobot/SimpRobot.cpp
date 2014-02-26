/*

   Simple example of robot Sense, Plan, Act loop.

 */

#include <iostream>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>    // for reference counting pointers
#include <Eigen/Eigen>             // for vector maths
#include <pangolin/pangolin.h>     // for open GL state management
#include <SceneGraph/SceneGraph.h> // for open GL scene graph

#include <Utils/GetPot>                    // for command line parsing
//#include <Utils/CVarHelpers.h>           // for parsing Eigen vars as CVars
//#include <CVars/CVar.h>                  // for glconsole

//#include <RPG/Utils/InitCam.h>
//#include <RPG/Devices/Camera/CameraDevice.h>
//#include <RPG/Devices/Controllers/CarController.h>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/Utils/GetPot>
#include <PbMsgs/Logger.h>

#include <URDFParser/GenURIFromURDF.h>

using namespace std;
using namespace pangolin;
using namespace SceneGraph;
using namespace boost;

#define USAGE \
"USAGE: Robot -n <RobotName> -n <LocalSimName>\n"\
"      --URDFFileName, -n           Name of URDF for this Robot.\n"\

class KeyControl
{
    public:
        Eigen::Vector6d             m_command;
        double                      m_DesireVelocity;
        double                      m_DesireSteering;

        void LeftKey()
        {
            m_command(5,0)=m_command(5,0)-0.1;
        }

        void RightKey()
        {
            m_command(5,0)=m_command(5,0)+0.1;
        }

        void ForwardKey()
        {
            m_command(0,0)=m_command(0,0)+10.0;
        }

        void ReverseKey()
        {
            m_command(0,0)=m_command(0,0)-10.0;
        }
};

///////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    // parse command line arguments
    GetPot cl( argc, argv );
    std::string sRobotURDF = cl.follow("/Users/malu/Code/RobotGroup/Simba/urdf/Robot.xml",1,"-r");

    if( argc != 3 ){
        puts(USAGE);
        return -1;
    }

    // Create OpenGL window in single line thanks to GLUT
    pangolin::CreateGlutWindowAndBind("Simple Robot",1280,480);
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
    glewInit();

    // sinle application context holds everything
    pangolin::View& container = pangolin::DisplayBase();

    // We define a special type of view which will accept image data
    // to display and set its bounds on screen.
    ImageView viewRGB(true,false);
    viewRGB.SetBounds(0.0, 0.1, 0, 0.5, (double)640/480);

    ImageView viewDepth(true,false);
    viewDepth.SetBounds(0.0, 0.1, 0.5, 1.0, (double)640/480);

    container.AddDisplay(viewRGB);
    container.AddDisplay(viewDepth);

/// Key
    // register a keyboard hook to trigger the reset method
    // simple asdw control
    KeyControl                  mKeyControl;
    RegisterKeyPressCallback( 'a', bind( &KeyControl::LeftKey, &mKeyControl ) );
    RegisterKeyPressCallback( 'A', bind( &KeyControl::LeftKey, &mKeyControl ) );

    RegisterKeyPressCallback( 's', bind( &KeyControl::ReverseKey, &mKeyControl ) );
    RegisterKeyPressCallback( 'S', bind( &KeyControl::ReverseKey, &mKeyControl ) );

    RegisterKeyPressCallback( 'd', bind( &KeyControl::RightKey, &mKeyControl ) );
    RegisterKeyPressCallback( 'D', bind( &KeyControl::RightKey, &mKeyControl ) );

    RegisterKeyPressCallback( 'w', bind( &KeyControl::ForwardKey, &mKeyControl ) );
    RegisterKeyPressCallback( 'W', bind( &KeyControl::ForwardKey, &mKeyControl ) );

    // Main
    // --------------------------------------------------------------------------------------------------------------------------------------------
    hal::Camera camera(GenURIFromURDF("LCamera", sRobotURDF));

//    rpg::CarController          Controler;

    pb::ImageArray imgs;
    while( !pangolin::ShouldQuit() )
    {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // ACT: Send commands to the robot
//      if (app.PublishCommandToLocalSim()==true)
//      {

        if( !camera.Capture(imgs) )
        {
           std::cout << "Error getting images." << std::endl;
        }

        viewRGB.SetImage(imgs[0].data(), camera.Width(), camera.Height(), GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE); //show RGB image
        viewDepth.SetImage(imgs[1].data(), camera.Width(), camera.Height(), GL_INTENSITY, GL_LUMINANCE, GL_FLOAT); //show depth image

        pangolin::FinishGlutFrame();
    }

    return 0;
}

