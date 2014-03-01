/*
   LocalSim, By luma. 2013.03
   Edited by bminortx.
 */

#include "LocalSim.h"

using namespace std;
using namespace CVarUtils;

////////////////////////////////////////////////////////////////////////
/// CONSTRUCTOR
/// LocalSim does not get produced willy-nilly; it is only produced if there
/// is a SIM.
////////////////////////////////////////////////////////////////////////

LocalSim::LocalSim(const std::string& sLocalSimName,
                   const std::string& sRobotURDFPath,
                   const std::string& sWorldURDFPath,
                   const std::string& sServerOption):
  m_sLocalSimName(sLocalSimName), m_SimDevices(&m_Scene)
{
  // 1. Read URDF files.
  XMLDocument RobotURDF, WorldURDF;
  GetXMLdoc(sRobotURDFPath, RobotURDF);
  GetXMLdoc(sWorldURDFPath, WorldURDF);

  // 2. Parse our world and our robot for objects in the scene.
  m_Parser.ParseWorld(WorldURDF, m_SimWorld);
  m_Parser.ParseDevices(RobotURDF, m_SimDevices, sLocalSimName);
  m_Parser.ParseRobot(RobotURDF, m_SimRobot, sLocalSimName);

  // 3. Init User's Robot and add it to RobotManager
  m_RobotManager.Init(m_sLocalSimName, m_Scene, m_SimRobot, sServerOption);

  // Do we want to run in debug mode?
  bool debug = false;

  // 5. Add the world, robot, and controllers to the ModelGraph
  m_Scene.Init(m_SimWorld, m_SimRobot, m_SimDevices,
               sLocalSimName, debug);

  // 4. We must decide the next actions based off of the Server Option.
  cout<<" The server option is set to "<<sServerOption<<"."<<endl;

  m_SimDevices.InitAllDevices();

  m_NetworkManager.Init( m_sLocalSimName, sServerOption);
  m_NetworkManager.RegisterRobot(&m_RobotManager);
  m_NetworkManager.RegisterDevices(&m_SimDevices);

  // TODO: What to do with StateKeeper option...?

  cout<<"[LocalSim] Init Local Sim Success!"<<endl;
}

////////////////////////////////////////////////////////////////////////
/// FUNCTIONS
////////////////////////////////////////////////////////////////////////

// Scan all SimDevices and send the simulated camera images to Pangolin.
// Right now, we can only support up to two windows.

//TODO: Fairly certain that there's a glitch here. Go through and correct the
//Image buffers for content.
bool LocalSim::SetImagesToWindow(SceneGraph::ImageView& LSimCamWnd,
                                 SceneGraph::ImageView& RSimCamWnd ){
  int WndCounter = 0;

  for(unsigned int i =0 ; i!= m_SimDevices.m_vSimDevices.size(); i++){
    SimDeviceInfo Device = m_SimDevices.m_vSimDevices[i];
    if(Device.m_bDeviceOn==true){
      for(unsigned int j=0;j!=Device.m_vSensorList.size();j++){
        string sSimCamName = Device.m_vSensorList[j];
        SimCamera* pSimCam = m_SimDevices.GetSimCam(sSimCamName);
        SceneGraph::ImageView* ImageWnd;
        // get pointer to window
        if(WndCounter == 0){
          ImageWnd = &LSimCamWnd;
        }
        else if(WndCounter == 1){
          ImageWnd = &RSimCamWnd;
        }

        WndCounter++;
        // set image to window
        if (pSimCam->m_iCamType == 5){       // for depth image
          float* pImgbuf = (float*) malloc( pSimCam->m_nImgWidth *
                                            pSimCam->m_nImgHeight *
                                            sizeof(float) );
          if(pSimCam->capture(pImgbuf)==true){
            ImageWnd->SetImage(pImgbuf, pSimCam->m_nImgWidth,
                               pSimCam->m_nImgHeight,
                               GL_INTENSITY, GL_LUMINANCE, GL_FLOAT);
            free(pImgbuf);
          }
          else{
            cout<<"[SetImagesToWindow] Set depth Image fail"<<endl;
            return false;
          }
        }
        else if(pSimCam->m_iCamType == 2){   // for RGB image
          char* pImgbuf= (char*)malloc (pSimCam->m_nImgWidth *
                                        pSimCam->m_nImgHeight * 3);
          if(pSimCam->capture(pImgbuf)==true){
            ImageWnd->SetImage(pImgbuf, pSimCam->m_nImgWidth,
                               pSimCam->m_nImgHeight,
                               GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE);
            free(pImgbuf);
          }
          else{
            cout<<"[SetImagesToWindow] Set RGB Image fail"<<endl;
            return false;
          }
        }
        else if(pSimCam->m_iCamType == 1){    //to show greyscale image
          char* pImgbuf= (char*)malloc (pSimCam->m_nImgWidth *
                                        pSimCam->m_nImgHeight);
          if(pSimCam->capture(pImgbuf)==true){
            ImageWnd->SetImage(pImgbuf, pSimCam->m_nImgWidth,
                               pSimCam->m_nImgHeight,
                               GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE);
            free(pImgbuf);
          }
          else{
            cout<<"[SetImagesToWindow] Set Gray Image fail"<<endl;
            return false;
          }
        }
      }
    }
  }
  return true;
}

////////////////////////////////

void LocalSim::StepForward(){

  // Update the PhysicsEngine and RenderEngine
  // Comes after UpdateNetwork due to controller commands
  m_Scene.UpdateScene();

  // Update SimDevices
  m_SimDevices.UpdateAllSensors();

  // Update the Network
  m_NetworkManager.UpdateNetwork();



  // Show the image in the current window
  SetImagesToWindow(*m_Scene.m_Render.m_LSimCamImage,
                    *m_Scene.m_Render.m_RSimCamImage);
}


///////////////////////////////////////////////////////////////////
/////
///// MAIN LOOP:
///// 1. Parses arguments in xml file
///// 2. Develops the ModelGraph and Physics Engine
///// 3. Renders the objects in the Sim.
/////
///////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  // parse command line arguments
  if(argc){
    std::cout<<"USAGE: Robot -n <LocalSimName> -r <robot.xml directory>"<<
               "-w <world.xml directory> -s <StateKeeper Option>"<<std::endl;
    std::cout<<"Options:"<<std::endl;
    std::cout<<"--LocalSimName, -n      ||   Name of this LocalSim."
            <<std::endl;
    std::cout<<"--Robot.xml, -r         ||   robot.xml's directory"
            <<std::endl;
    std::cout<<"--World.xml, -w         ||   world.xml's directory."
            <<std::endl;
    std::cout<<"--Statekeeper Option -s ||   input 'StateKeeperName',"<<
               " 'WithoutStateKeeper', or 'WithoutNetwork'"<<std::endl;

  }

  GetPot cl( argc, argv );
  std::string sLocalSimName = cl.follow( "SimWorld", "-n" );
  std::string sRobotURDF = cl.follow("", "-r");
  std::string sWorldURDF = cl.follow( "", "-w" );
  std::string sServerOption = cl.follow("WithoutStateKeeper", "-s");

  // Initialize a LocalSim.
  LocalSim mLocalSim(sLocalSimName, sRobotURDF, sWorldURDF, sServerOption);

  pangolin::RegisterKeyPressCallback('s',
                                     boost::bind( &LocalSim::StepForward,
                                                  &mLocalSim) );

  // Default hooks for exiting (Esc) and fullscreen (tab).
  while( !pangolin::ShouldQuit() ){
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    // Swap frames and Process Events
    pangolin::FinishFrame();

    // Update Physics and ModelGraph
    mLocalSim.StepForward();

    usleep( 1E6 / 60 );
  }

  return 0;

}
