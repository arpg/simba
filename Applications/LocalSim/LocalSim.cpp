#include "LocalSim.h"

using namespace std;
using namespace CVarUtils;

////////////////////////////////////////////////////////////////////////
/// CONSTRUCTOR
////////////////////////////////////////////////////////////////////////

LocalSim::LocalSim(const string& sLocalSimName,
                   const string& sRobotURDFPath,
                   const string& sWorldURDFPath,
                   const string& sServerOption):
    m_sLocalSimName(sLocalSimName) {
  // Do we want to run in debug mode?
  // 0 = yes, 1 = no
  int debug_level = 0;
  // Do we want to render the world?
  m_bRender = true;

  // 1. Read URDF files.
  XMLDocument RobotURDF, WorldURDF;
  GetXMLdoc(sRobotURDFPath, RobotURDF);
  GetXMLdoc(sWorldURDFPath, WorldURDF);

  // 2. Parse our world and our robot for objects in the scene.
  parser_ = new URDF_Parser(debug_level);
  parser_->ParseWorld(WorldURDF, m_SimWorld);
  parser_->ParseDevices(RobotURDF, m_SimDevices, sLocalSimName);
  parser_->ParseRobot(RobotURDF, m_SimRobot, sLocalSimName);

  // 3. Init User's Robot and add it to RobotManager
  m_RobotManager.Init(m_sLocalSimName, m_Scene, m_SimRobot, sServerOption);



  // 4. We must decide the next actions based off of the Server Option.
  LOG(debug_level) << " The server option is set to " << sServerOption << ".";

  m_NetworkManager.Init(m_sLocalSimName, sServerOption, debug_level);
  m_NetworkManager.RegisterRobot(&m_RobotManager);
  m_NetworkManager.RegisterDevices(&m_SimDevices);

  // 5. Add the world, robot, and controllers to the ModelGraph
  m_Scene.Init(m_SimWorld, m_SimRobot, m_SimDevices,
               sLocalSimName, false, m_bRender, false);

  // TODO: What to do with StateKeeper option...?
  LOG(debug_level) << "Init Local Sim Success!";

}

////////////////////////////////////////////////////////////////////////
/// FUNCTIONS
////////////////////////////////////////////////////////////////////////

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
  if (argc != 9) {
    LOG(INFO) << " Command Line Arguments: ";
    LOG(INFO) << "   $ ./LocalSim -n <SimName> -r <Robot.xml>"
              << " -w <World.xml> -s <StateKeeper Option>";
    LOG(INFO) << " See the README for more info.";
  }
  else{
    GetPot cl( argc, argv );
    std::string sLocalSimName = cl.follow( "SimWorld", "-n" );
    std::string sRobotURDF = cl.follow("", "-r");
    std::string sWorldURDF = cl.follow( "", "-w" );
    std::string sServerOption = cl.follow("WithoutStateKeeper", "-s");

    // Initialize a LocalSim.
    LocalSim mLocalSim(sLocalSimName, sRobotURDF, sWorldURDF, sServerOption);

    // Are we rendering the world?
    if(mLocalSim.m_bRender){
      while( !pangolin::ShouldQuit() ){
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        // Swap frames and Process Events
        pangolin::FinishFrame();
        // Update Physics and ModelGraph
        mLocalSim.StepForward();
        usleep( 1E6 / 60 );
      }
    }

    else{
      while(1){
        mLocalSim.StepForward();
      }
    }
  }
  return 0;
}
