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
//#include <Utils/CVarHelpers.h>           // for parsing Eigen vars as CVars
//#include <CVars/CVar.h>                  // for glconsole

//#include <RPG/Utils/InitCam.h>
//#include <RPG/Devices/Camera/CameraDevice.h>
//#include <RPG/Devices/Controllers/CarController.h>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/Utils/GetPot>
#include <PbMsgs/Logger.h>

#include <src/URDFParser/GenURIFromURDF.h>

using namespace std;
using namespace pangolin;
using namespace SceneGraph;
using namespace boost;

#define USAGE \
"USAGE: Robot -n <RobotName> -n <RobotProxyName>\n"\
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
    std::string sRobotURDF = cl.follow("/Users/malu/Code/Luma/Sim-HAL/urdf/Robot.xml",1,"-r");

    if( argc != 3 ){
        puts(USAGE);
        return -1;
    }
    // Create OpenGL window in single line thanks to GLUT
    pangolin::CreateGlutWindowAndBind("Simple Car",640,480);
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
    glewInit();

    // sinle application context holds everything
    pangolin::View& container = pangolin::DisplayBase();

    // We define a special type of view which will accept image data
    // to display and set its bounds on screen.
    ImageView viewImage(true,false);
    viewImage.SetBounds(0.0, 1.0, 0, 1.0, (double)640/480);
    container.AddDisplay(viewImage);

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
    hal::Camera camera(GenURIFromURDF("RCamera", sRobotURDF));

//    rpg::CarController          Controler;

    pb::ImageArray imgs;
    while( !pangolin::ShouldQuit() )
    {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // ACT: Send commands to the robot
//      if (app.PublishCommandToRobotProxy()==true)
//      {

        if( !camera.Capture(imgs) )
        {
           std::cout << "Error getting images." << std::endl;
        }

//      viewImage.SetImage(vImages[0].Image.data, vImages[0].width(), vImages[0].height(), GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE); //show RGB image
//        viewImage.SetImage(imgs[0], camera.Width(), camera.Height(), GL_INTENSITY, GL_LUMINANCE, GL_FLOAT); //show depth image

//        for(size_t i=0; i<imgs.Size(); ++i ) {
//            container[i].Activate();
//            tex.Upload(
//                imgs[i].data(),
//                imgs[i].Format(), imgs[i].Type()
//            );
//            tex.RenderToViewportFlipY();
//        }

//        if(log && run) {
//            pb::Msg msg;
//            msg.set_timestamp(frame);
//            msg.mutable_camera()->Swap(&imgs.Ref());
//            pb::Logger::GetInstance().LogMessage(msg);
//        }

//        usleep(1E6/30);

        pangolin::FinishGlutFrame();
    }

    return 0;
}

