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

LocalSim::LocalSim(const string& sLocalSimName,
                   const string& sRobotURDFPath,
                   const string& sWorldURDFPath,
                   const string& sServerOption):
  m_sLocalSimName(sLocalSimName)
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

  m_NetworkManager.Init( m_sLocalSimName, sServerOption);
  m_NetworkManager.RegisterRobot(&m_RobotManager);
  m_NetworkManager.RegisterDevices(&m_SimDevices);

  // TODO: What to do with StateKeeper option...?

  cout<<"[LocalSim] Init Local Sim Success!"<<endl;
}

////////////////////////////////////////////////////////////////////////
/// FUNCTIONS
////////////////////////////////////////////////////////////////////////



////////////////////////////////

void LocalSim::StepForward(){
  // Update SimDevices
  m_SimDevices.UpdateSensors();
  // Update the Network
  m_NetworkManager.UpdateNetwork();
  // Update the PhysicsEngine and RenderEngine
  // Comes after UpdateNetwork due to controller commands
  m_Scene.UpdateScene();
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
