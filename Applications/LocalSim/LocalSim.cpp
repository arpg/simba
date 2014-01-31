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
  m_sLocalSimName (sLocalSimName)
{
  // 1. Read Robot.xml file.
  XMLDocument RobotURDF, WorldURDF;
  GetXMLdoc(sRobotURDFPath, RobotURDF);
  GetXMLdoc(sWorldURDFPath, WorldURDF);

  // 2. Parse world.xml file.
  if( m_Parser.ParseWorld(WorldURDF, m_WorldManager) == false){
    cout<<"[LocalSim] Parse World Fail."<<endl;
    exit(-1);
  }

  // 3. Init User's Robot and add it to RobotManager
  if(m_RobotManager.Init(
       m_sLocalSimName, sServerName, m_Scene, RobotURDF) == false)
  {
    cout<<"[LocalSim] Init Robot Fail."<<endl;
    exit(-1);
  }

  m_pMainRobot = m_RobotManager.GetMainRobot();

  // 4. Add the robot to the Model Graph Scene
  m_Scene.Init(ModelGraphBuilder::All,m_pMainRobot->GetRobotModel(),sLocalSimName);

  // 5. Initialize the Sim Device (SimCam, SimGPS, SimVicon, etc...)
  if( m_SimDeviceManager.InitFromXML(
        m_Scene.m_Phys,m_Scene.m_Render.m_glGraph,
        RobotURDF,m_sLocalSimName, sPoseFileName) == false)
  {
    cout<<"[LocalSim] Init SimDevice Fail."<<endl;
    exit(-1);
  }

  // 6. Initialize the Network
  if(m_NetworkManager.InitNetwork(
       m_sLocalSimName, &m_RobotManager, sServerName) ==false)
  {
    cout<<"[LocalSim] Init Network Fail."<<endl;
    exit(-1);
  }

  // 7, if run in with network mode, LocalSim network will publish sim device
  m_NetworkManager.CheckIfInitDevices(&m_SimDeviceManager);

  cout<<"[LocalSim] Init Local Sim Success!"<<endl;
}

////////////////////////////////////////////////////////////////////////
/// FUNCTIONS
////////////////////////////////////////////////////////////////////////

// InitReset will populate SceneGraph with objects, and
// register these objects with the simulator.
void LocalSim::InitReset()
{
  SceneGraph::GLLight*        m_light = new SceneGraph::GLLight();
  SceneGraph::GLBox*          m_ground = new SceneGraph::GLBox();
  SceneGraph::GLGrid*         m_grid = new SceneGraph::GLGrid();
  SceneGraph::GLMesh*         m_Map = new SceneGraph::GLMesh();                 // mesh for the world.

  m_Scene.m_Render.m_glGraph.Clear();

  m_light->SetPosition( m_WorldManager.vLightPose[0],
      m_WorldManager.vLightPose[1],
      m_WorldManager.vLightPose[2]);
  m_Scene.m_Render.m_glGraph.AddChild( m_light );

  // init world without mesh
  if (m_WorldManager.m_sMesh =="NONE")
  {
    cout<<"[RobotRroxy] Try init empty world."<<endl;
    m_grid->SetNumLines(20);
    m_grid->SetLineSpacing(1);
    m_Scene.m_Render.m_glGraph.AddChild(m_grid);

    double dThickness = 1;
    m_ground->SetPose( 0,0, dThickness/2.0,0,0,0 );
    m_ground->SetExtent( 200,200, dThickness );
    m_Scene.m_Render.m_glGraph.AddChild( m_ground );

    BoxShape* pGround = new BoxShape("Ground",100, 100, 0.01, 0, 1, m_WorldManager.vWorldPose);
    m_Scene.m_Phys.RegisterObject(pGround);
    m_Scene.m_Phys.SetFriction("Ground", 888);
  }
  // init world with mesh
  // maybe dangerous to always reload meshes?
  /// maybe we should separate Init from Reset?
  else {
    try
    {
      cout<<"[RobotRroxy] Try init word with mesh: "
         <<m_WorldManager.m_sMesh<<endl;
      m_Map->Init(m_WorldManager.m_sMesh);
      m_Map->SetPerceptable(true);
      m_Map->SetScale(m_WorldManager.iScale);
      m_Map->SetPosition(m_WorldManager.vWorldPose[0],
          m_WorldManager.vWorldPose[1],
          m_WorldManager.vWorldPose[2]);
      m_Scene.m_Render.m_glGraph.AddChild( m_Map );
    } catch (std::exception e) {
      printf( "Cannot load world map\n");
      exit(-1);
    }
  }

  m_Scene.m_Render.AddToScene();
  cout<<"[LocalSim/InitReset] Init World Success"<<endl;
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

  for(unsigned int i =0 ; i!= m_SimDeviceManager.m_vSimDevices.size(); i++){
    SimDeviceInfo Device = m_SimDeviceManager.m_vSimDevices[i];

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

// ---- Step Forward
void LocalSim::StepForward( void )
{
  m_Scene.m_Phys.StepSimulation();
  m_Scene.m_Render.UpdateScene();
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

  pangolin::RegisterKeyPressCallback(
        ' ', boost::bind( &LocalSim::StepForward, &mLocalSim ) );

  // Default hooks for exiting (Esc) and fullscreen (tab).
  while( !pangolin::ShouldQuit() )
  {
    // 2. Update physics and scene
    mLocalSim.m_Scene.UpdateScene(true);

    // 3. Update SimDevices
    mLocalSim.m_SimDeviceManager.UpdateAllDevices();

    // 4. Update the Network
    mLocalSim.m_NetworkManager.UpdateNetWork();

    // 5. Show the image in the current window
    mLocalSim.SetImagesToWindow(*mLocalSim.m_Scene.m_Render.m_LSimCamImage,
                                *mLocalSim.m_Scene.m_Render.m_RSimCamImage);

    // 6. Refresh screen
    pangolin::FinishGlutFrame();
    usleep( 1E6 / 60 );
  }

  return 0;

}

