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
    const std::string& sRobotURDF,      //< Input: location of meshes, maps etc
    const std::string& sWorldURDF,
    const std::string& sServerName,
    const std::string& sPoseFileName)
  : m_sLocalSimName(sLocalSimName),
    m_sWorldURDFFile(sWorldURDF),
    m_sRobotURDFFile(sRobotURDF)
{
  // 1. Parse world.xml file.
  if( m_Parser.ParseWorld(m_sWorldURDFFile.c_str(), m_WorldManager) != true){
    cout<<"[LocalSim] Cannot parse "<< m_sWorldURDFFile<<
          ". Please check if the file exist and the syntax is valid."<<endl;
    exit(-1);
  }

  // 2. Read Robot.xml file.
  XMLDocument RobotURDF;
  GetXMLdoc(sRobotURDF, RobotURDF);

  // 4. Init User's Robot and add it to RobotManager
  m_RobotManager.Init(m_sLocalSimName, sServerName, m_Scene);
  m_RobotManager.BuildRobot(RobotURDF, m_sLocalSimName);
  m_pMainRobot = m_RobotManager.GetMainRobot();
  // Add the robot to the Model Graph Scene
  m_Scene.Init(ModelGraphBuilder::All, m_pMainRobot->GetRobotModel(),
               sLocalSimName);

  // 5. Initialize the Network
  m_NetworkManager.initNetwork(m_sLocalSimName, &m_SimDeviceManager,
                               &m_RobotManager,  sServerName);

  // 6. Initialize the Sim Device (SimCam, SimGPS, SimVicon, etc...)

  // This has to be fixed up (there are a lot of references that shouldn't be
  // there), but we'll run with it for now.
  m_SimDeviceManager.Init(m_Scene.m_Phys, m_Scene.m_Render.glGraph,
                          RobotURDF, m_sLocalSimName);
  m_SimpPoseController.Init(sPoseFileName);

  // 7, if run in with network mode, LocalSim network will publish sim device
  if(sServerName !="WithoutNetwork"){
    if(m_NetworkManager.initDevices()!=true){
      cout<<"[LocalSim] Cannot init Nextwrok"<<endl;
      exit(-1);
    }
  }
  else
  {
    cout<<"[LocalSim] Init Robot LocalSim without Network."<<endl;
  }

}

////////////////////////////////////////////////////////////////////////
/// FUNCTIONS
////////////////////////////////////////////////////////////////////////

// InitReset will populate SceneGraph with objects, and
// register these objects with the simulator.
void LocalSim::InitReset()
{
    SceneGraph::GLLight         m_light;
    SceneGraph::GLBox           m_ground;
    SceneGraph::GLGrid          m_grid;
    SceneGraph::GLMesh          m_Map;                 // mesh for the world.

  m_Scene.m_Render.glGraph.Clear();

//  m_light.SetPosition( m_WorldManager.vLightPose[0],
//                       m_WorldManager.vLightPose[1],
//                       m_WorldManager.vLightPose[2]);
//  m_Scene.m_Render.glGraph.AddChild( &m_light );

  // init world without mesh
  if (m_WorldManager.m_sMesh =="NONE")
  {
    cout<<"[RobotRroxy] Try init empty world."<<endl;
    m_grid.SetNumLines(20);
    m_grid.SetLineSpacing(1);
//    m_Scene.m_Render.glGraph.AddChild(&m_grid);

//    double dThickness = 1;
//    m_ground.SetPose( 0,0, dThickness/2.0,0,0,0 );
//    m_ground.SetExtent( 200,200, dThickness );
//    m_Scene.m_Render.glGraph.AddChild( &m_ground );

//    BoxShape* pGround = new BoxShape("Ground",100, 100, 0.01, 0, 1, m_WorldManager.vWorldPose);
//    m_Scene.m_Phys.RegisterObject(pGround);
//    m_Scene.m_Phys.SetFriction("Ground", 888);
  }
  // init world with mesh
  // maybe dangerous to always reload meshes?
  /// maybe we should separate Init from Reset?
  else {
    try
    {
      cout<<"[RobotRroxy] Try init word with mesh: "
         <<m_WorldManager.m_sMesh<<endl;
      m_Map.Init(m_WorldManager.m_sMesh);
      m_Map.SetPerceptable(true);
      m_Map.SetScale(m_WorldManager.iScale);
      m_Map.SetPosition(m_WorldManager.vWorldPose[0],
                        m_WorldManager.vWorldPose[1],
                        m_WorldManager.vWorldPose[2]);
      m_Scene.m_Render.glGraph.AddChild( &m_Map );
    } catch (std::exception e) {
      printf( "Cannot load world map\n");
      exit(-1);
    }
  }

  m_Scene.m_Render.AddToScene();
  cout<<"Init World Success"<<endl;
}

//////////////////////////////////////////////////////////////////
// Apply the camera's pose directly to the SimCamera
void LocalSim::ApplyCameraPose(Eigen::Vector6d dPose){
  Eigen::Vector6d InvalidPose;
  InvalidPose<<1,88,99,111,00,44;
  if(dPose == InvalidPose){
  }
  else{
    string sMainRobotName = m_pMainRobot->GetRobotName();

    // update RGB camera pose
    string sNameRGBCam   = "RGBLCamera@" + sMainRobotName;
    m_SimDeviceManager.GetSimCam(sNameRGBCam)->UpdateByPose(_Cart2T(dPose));

    // update Depth camera pose
    string sNameDepthCam = "DepthLCamera@"+sMainRobotName;
    m_SimDeviceManager.GetSimCam(sNameDepthCam)->UpdateByPose(_Cart2T(dPose));
  }
}

void LocalSim::ApplyPoseToEntity(string sName, Eigen::Vector6d dPose){
  m_Scene.m_Phys.SetEntity6Pose(sName, dPose);
}

//////////////////////////////////////////////////////////////////
// Scan all SimDevices and send the simulated camera images to Pangolin.
// Right now, we can only support up to two windows.
bool LocalSim::SetImagesToWindow(SceneGraph::ImageView& LSimCamWnd, SceneGraph::ImageView& RSimCamWnd ){
  int WndCounter = 0;

  for(unsigned int i =0 ; i!= m_SimDeviceManager.m_SimDevices.size(); i++){
    SimDeviceInfo Device = m_SimDeviceManager.m_SimDevices[i];

    for(unsigned int j=0;j!=Device.m_vSensorList.size();j++){

      string sSimCamName = Device.m_vSensorList[j];
      SimCam* pSimCam = m_SimDeviceManager.GetSimCam(sSimCamName);

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
        float* pImgbuf = (float*) malloc( pSimCam->g_nImgWidth *
                                          pSimCam->g_nImgHeight *
                                          sizeof(float) );

        if(pSimCam->capture(pImgbuf)==true){
          ImageWnd->SetImage(pImgbuf, pSimCam->g_nImgWidth,
                             pSimCam->g_nImgHeight,
                             GL_INTENSITY, GL_LUMINANCE, GL_FLOAT);
          free(pImgbuf);
        }
        else{
          cout<<"[SetImagesToWindow] Set depth Image fail"<<endl;
          return false;
        }
      }
      else if(pSimCam->m_iCamType == 2){   // for RGB image
        char* pImgbuf= (char*)malloc (pSimCam->g_nImgWidth *
                                      pSimCam->g_nImgHeight * 3);

        if(pSimCam->capture(pImgbuf)==true){
          ImageWnd->SetImage(pImgbuf, pSimCam->g_nImgWidth,
                             pSimCam->g_nImgHeight,
                             GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE);
          free(pImgbuf);
        }
        else{
          cout<<"[SetImagesToWindow] Set RGB Image fail"<<endl;
          return false;
        }
      }
      else if(pSimCam->m_iCamType == 1){    //to show greyscale image
        char* pImgbuf= (char*)malloc (pSimCam->g_nImgWidth *
                                      pSimCam->g_nImgHeight);
        if(pSimCam->capture(pImgbuf)==true){
          ImageWnd->SetImage(pImgbuf, pSimCam->g_nImgWidth,
                             pSimCam->g_nImgHeight,
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

////////////////////
/// INPUT KEYS
////////////////////

void LocalSim::LeftKey(){
  string sMainRobotName = m_pMainRobot->GetRobotName();

  // update RGB camera pose
  string sNameRGBCam   = "RGBLCamera@" + sMainRobotName;

  Eigen::Vector6d dPoseRGB = _T2Cart(m_SimDeviceManager.GetSimCam(sNameRGBCam)->GetCameraPose() );
  dPoseRGB(5,0) = dPoseRGB(5,0) - 0.1;
  m_SimDeviceManager.GetSimCam(sNameRGBCam)->UpdateByPose(_Cart2T(dPoseRGB));

  // update Depth camera pose
  string sNameDepthCam = "DepthLCamera@"+sMainRobotName;

  Eigen::Vector6d dPoseDepth =
      _T2Cart(m_SimDeviceManager.GetSimCam(sNameDepthCam)->GetCameraPose() );
  dPoseDepth(5,0) = dPoseDepth(5,0) - 0.1;
  m_SimDeviceManager.
      GetSimCam(sNameDepthCam)->UpdateByPose(_Cart2T(dPoseDepth));

  //            string sMainRobotName = m_pMainRobot->GetRobotName();
  //            string sName = "RCamera@" + sMainRobotName;
  //            Eigen::Vector6d dPose;
  //            m_PhysEngine.GetEntity6Pose(sName, dPose);
  //            dPose(0,0) = dPose(0,0) + 1;
  //            m_PhysEngine.SetEntity6Pose(sName, dPose);
}

void LocalSim::RightKey(){
  string sMainRobotName = m_pMainRobot->GetRobotName();

  // update RGB camera pose
  string sNameRGBCam   = "RGBLCamera@" + sMainRobotName;

  Eigen::Vector6d dPoseRGB = _T2Cart(m_SimDeviceManager.GetSimCam(sNameRGBCam)->GetCameraPose() );
  dPoseRGB(5,0) = dPoseRGB(5,0) + 0.1;
  m_SimDeviceManager.GetSimCam(sNameRGBCam)->UpdateByPose(_Cart2T(dPoseRGB));

  // update Depth camera pose
  string sNameDepthCam = "DepthLCamera@"+sMainRobotName;

  Eigen::Vector6d dPoseDepth =
      _T2Cart(m_SimDeviceManager.GetSimCam(sNameDepthCam)->GetCameraPose() );
  dPoseDepth(5,0) = dPoseDepth(5,0) + 0.1;
  m_SimDeviceManager.
      GetSimCam(sNameDepthCam)->UpdateByPose(_Cart2T(dPoseDepth));
}

void LocalSim::ForwardKey(){
  //  you should update the pose of the rig and then the poses of the cameras would always be relative to the rig
  string sMainRobotName = m_pMainRobot->GetRobotName();

  // update RGB camera pose
  string sNameRGBCam   = "RGBLCamera@" + sMainRobotName;

  Eigen::Vector6d dPoseRGB = _T2Cart( m_SimDeviceManager.GetSimCam(sNameRGBCam)->GetCameraPose() );
  dPoseRGB(1,0) = dPoseRGB(1,0) + 1;
  m_SimDeviceManager.GetSimCam(sNameRGBCam)->UpdateByPose(_Cart2T(dPoseRGB));

  // update Depth camera pose
  string sNameDepthCam = "DepthLCamera@"+sMainRobotName;

  Eigen::Vector6d dPoseDepth = _T2Cart(m_SimDeviceManager.GetSimCam(sNameDepthCam)->GetCameraPose() );
  dPoseDepth(1,0) = dPoseDepth(1,0) + 1;
  m_SimDeviceManager.GetSimCam(sNameDepthCam)->UpdateByPose(_Cart2T(dPoseDepth));
}

void LocalSim::ReverseKey(){
  string sMainRobotName = m_pMainRobot->GetRobotName();

  // update RGB camera pose
  string sNameRGBCam   = "RGBLCamera@" + sMainRobotName;

  Eigen::Vector6d dPoseRGB = _T2Cart(m_SimDeviceManager.GetSimCam(sNameRGBCam)->GetCameraPose() );
  dPoseRGB(1,0) = dPoseRGB(1,0) - 1;
  m_SimDeviceManager.GetSimCam(sNameRGBCam)->UpdateByPose(_Cart2T(dPoseRGB));

  // update Depth camera pose
  string sNameDepthCam = "DepthLCamera@"+sMainRobotName;

  Eigen::Vector6d dPoseDepth = _T2Cart(m_SimDeviceManager.GetSimCam(sNameDepthCam)->GetCameraPose() );
  dPoseDepth(1,0) = dPoseDepth(1,0) - 1;
  m_SimDeviceManager.
      GetSimCam(sNameDepthCam)->UpdateByPose(_Cart2T(dPoseDepth));
}


///////////////////////////////////////////////////////////////////
/////
///// MAIN LOOP:
///// 1. Parses arguments in xml file
///// 2. Develops the SceneGraph
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
  mLocalSim.InitReset();

  //////KEYBOARD COMMANDS
//  pangolin::RegisterKeyPressCallback( pangolin::PANGO_CTRL + 'r',
//                                      boost::bind( &LocalSim::InitReset,
//                                                   &mLocalSim ) );
  pangolin::RegisterKeyPressCallback(
        'a', boost::bind( &LocalSim::LeftKey, &mLocalSim ) );
  pangolin::RegisterKeyPressCallback(
        'A', boost::bind( &LocalSim::LeftKey, &mLocalSim ) );

  pangolin::RegisterKeyPressCallback(
        's', boost::bind( &LocalSim::ReverseKey, &mLocalSim ) );
  pangolin::RegisterKeyPressCallback(
        'S', boost::bind( &LocalSim::ReverseKey, &mLocalSim ) );

  pangolin::RegisterKeyPressCallback(
        'd', boost::bind( &LocalSim::RightKey, &mLocalSim ) );
  pangolin::RegisterKeyPressCallback(
        'D', boost::bind( &LocalSim::RightKey, &mLocalSim ) );

  pangolin::RegisterKeyPressCallback(
        'w', boost::bind( &LocalSim::ForwardKey, &mLocalSim ) );
  pangolin::RegisterKeyPressCallback(
        'W', boost::bind( &LocalSim::ForwardKey, &mLocalSim ) );

//  pangolin::RegisterKeyPressCallback(
//        ' ', boost::bind( &LocalSim::StepForward, &mLocalSim ) );

  // Default hooks for exiting (Esc) and fullscreen (tab).
  while( !pangolin::ShouldQuit() )
  {
    // 2. Update physics and scene
    mLocalSim.m_Scene.UpdateScene();

    // 3. Update SimDevices
    mLocalSim.m_SimDeviceManager.UpdateAllDevices();

    // 4. Update the Network
    mLocalSim.m_NetworkManager.UpdateNetWork();

    // 5. Show the image in the current window
    mLocalSim.SetImagesToWindow(*mLocalSim.m_Scene.m_Render.LSimCamImage,
                                *mLocalSim.m_Scene.m_Render.RSimCamImage);

    // 6. Refresh screen
    pangolin::FinishGlutFrame();
    usleep( 1E6 / 60 );
  }

  return 0;

}
