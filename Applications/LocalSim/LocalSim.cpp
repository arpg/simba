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

LocalSim::LocalSim(const std::string& sLocalSimName,      //< Input: name of robot LocalSim
                   const std::string& sRobotURDFPath,      //< Input: location of meshes, maps etc
                   const std::string& sWorldURDFPath,
                   const std::string& sServerName,
                   const std::string& sPoseFileName):
  m_sLocalSimName (sLocalSimName), m_SimDeviceManager(&m_Scene)
{
  // 1. Read URDF files.
  XMLDocument RobotURDF, WorldURDF;
  GetXMLdoc(sRobotURDFPath, RobotURDF);
  GetXMLdoc(sWorldURDFPath, WorldURDF);

  // 2. Parse our world and our robot for objects in the scene.
  m_Parser.ParseWorld(WorldURDF, m_SimWorld);
  m_Parser.ParseDevices(RobotURDF, m_SimDeviceManager, sLocalSimName);
  m_Parser.ParseRobot(RobotURDF, m_SimRobot, sLocalSimName);


  // 3. Init User's Robot and add it to RobotManager
  m_RobotManager.Init(m_sLocalSimName, m_Scene, m_SimRobot, sServerName);

  // 4. Add the world and robot to the Model Graph Scene
  m_Scene.Init(ModelGraphBuilder::All, m_SimWorld, m_SimRobot,sLocalSimName);

  m_SimDeviceManager.InitAllDevices();

  // 6. Initialize the Network
  m_NetworkManager.Init( m_sLocalSimName, sServerName);

  m_NetworkManager.PubRobotIfNeeded(&m_RobotManager);

  // 7, if run in with network mode, LocalSim network will publish sim device
  m_NetworkManager.PubRegisterDevicesIfNeeded(&m_SimDeviceManager);

  cout<<"[LocalSim] Init Local Sim Success!"<<endl;
}

////////////////////////////////////////////////////////////////////////
/// FUNCTIONS
////////////////////////////////////////////////////////////////////////

// Apply the camera's pose directly to the SimCamera
void LocalSim::ApplyCameraPose(Eigen::Vector6d dPose){
  Eigen::Vector6d InvalidPose;
  InvalidPose<<1,88,99,111,00,44;
  if(dPose == InvalidPose){
  }
  else{
    string sMainRobotName = m_SimRobot.GetRobotName();

    // update RGB camera pose
    string sNameRGBCam   = "RGBLCamera@" + sMainRobotName;
    m_SimDeviceManager.GetSimCam(sNameRGBCam)->UpdateByPose(_Cart2T(dPose));

    // update Depth camera pose
    string sNameDepthCam = "DepthLCamera@"+sMainRobotName;
    m_SimDeviceManager.GetSimCam(sNameDepthCam)->UpdateByPose(_Cart2T(dPose));
  }
}

/////////////

void LocalSim::ApplyPoseToEntity(string sName, Eigen::Vector6d dPose){
  m_Scene.m_Phys.SetEntity6Pose(sName, dPose);
}

//////////////////////////////////////////////////////////////////
// Scan all SimDevices and send the simulated camera images to Pangolin.
// Right now, we can only support up to two windows.
bool LocalSim::SetImagesToWindow(SceneGraph::ImageView& LSimCamWnd, SceneGraph::ImageView& RSimCamWnd ){
  int WndCounter = 0;

  for(unsigned int i =0 ; i!= m_SimDeviceManager.m_vSimDevices.size(); i++){
    SimDeviceInfo Device = m_SimDeviceManager.m_vSimDevices[i];

    for(unsigned int j=0;j!=Device.m_vSensorList.size();j++){

      string sSimCamName = Device.m_vSensorList[j];
      SimCamera* pSimCam = m_SimDeviceManager.GetSimCam(sSimCamName);

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
  return true;
}

// ---- Step Forward
void LocalSim::StepForward( bool debug )
{
  m_Scene.UpdateScene(debug);
  // Update SimDevices
  m_SimDeviceManager.UpdateAllDevices();

  // Update the Network
  m_NetworkManager.UpdateNetWork();

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
    std::cout<<"--LocalSimName, -n         ||   Name of this LocalSim."
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
  std::string sPoseFile  = cl.follow("None",1,"-p");
  std::string sServerOption = cl.follow("WithoutStateKeeper", "-s");

  // Initialize a LocalSim.
  LocalSim mLocalSim(sLocalSimName, sRobotURDF,
                     sWorldURDF, sServerOption, sPoseFile);

  // Run as debug?
  bool debug = false;

  pangolin::RegisterKeyPressCallback(
        ' ', boost::bind( &LocalSim::StepForward, &mLocalSim, &debug ) );

  // Default hooks for exiting (Esc) and fullscreen (tab).
  while( !pangolin::ShouldQuit() ){

    // Update Physics and ModelGraph
    mLocalSim.StepForward(debug);

    // Refresh screen
    pangolin::FinishGlutFrame();
    usleep( 1E6 / 60 );
  }

  return 0;

}

