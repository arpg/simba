#include "LocalSim.h"

using namespace std;
using namespace CVarUtils;

////////////////////////////////////////////////////////////////////////
/// CONSTRUCTOR
////////////////////////////////////////////////////////////////////////

LocalSim::LocalSim(const string& local_sim_name,
                   const string& robot_urdf_path,
                   const string& world_urdf_path,
                   const string& server_option,
                   int debug_level):
    local_sim_name_(local_sim_name) {

  // Do we want to render the world?
  render_option_ = true;

  // 1. Read URDF files.
  XMLDocument robot_xml, world_xml;
  GetXMLdoc(robot_urdf_path, robot_xml);
  GetXMLdoc(world_urdf_path, world_xml);

  // 2. Parse our world and our robot for objects in the scene.
  parser_ = new URDFParser(debug_level);
  parser_->ParseWorld(world_xml, sim_world_);
  parser_->ParseDevices(robot_xml, sim_devices_, local_sim_name_);
  parser_->ParseRobot(robot_xml, sim_robot_, local_sim_name_);

  // 3. Init User's Robot and add it to RobotManager
  robot_manager_.Init(local_sim_name_, scene_, sim_robot_, server_option);

  // 4. We must decide the next actions based off of the Server Option.
  LOG(debug_level) << " The server option is set to " << server_option << ".";
  network_manager_.Init(local_sim_name_, server_option, debug_level);
  network_manager_.RegisterRobot(&robot_manager_);
  network_manager_.RegisterDevices(&sim_devices_);

  // 5. Add the world, robot, and controllers to the ModelGraph
  scene_.Init(sim_world_, sim_robot_, sim_devices_,
              local_sim_name_, false, render_option_, false);

  // TODO: What to do with StateKeeper option...?
  LOG(debug_level) << std::endl
                   << "-------> SUCCESS: Initialized LocalSim! <-------";

}

////////////////////////////////////////////////////////////////////////
/// FUNCTIONS
////////////////////////////////////////////////////////////////////////

void LocalSim::StepForward(){
  // Update SimDevices
  sim_devices_.UpdateSensors();
  // Update the Network
  network_manager_.UpdateNetwork();
  // Update the PhysicsEngine and RenderEngine
  // Comes after UpdateNetwork due to controller commands
  scene_.UpdateScene();
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
  if (argc != 9 && argc != 10) {
    LOG(INFO) << " Command Line Arguments: ";
    LOG(INFO) << "   $ ./LocalSim -n <SimName> -r <Robot.xml>"
              << " -w <World.xml> -s <StateKeeper Option> <-debug>";
    LOG(INFO) << " See the README for more info.";
  } else {
    GetPot cl( argc, argv );
    std::string local_sim_name = cl.follow( "SimWorld", "-n" );
    std::string robot_urdf_path = cl.follow("", "-r");
    std::string world_urdf_path = cl.follow( "", "-w" );
    std::string server_option = cl.follow("WithoutStateKeeper", "-s");
    // Do we want to run in debug mode?
    // 0 = yes, 1 = no
    int debug_level = 1;
    if (argc == 10) {
      debug_level = 0;
    }

    // Initialize a LocalSim.
    LocalSim local_sim(local_sim_name, robot_urdf_path, world_urdf_path,
                       server_option, debug_level);

    // Are we rendering the world?
    if (local_sim.render_option_) {
      while (!pangolin::ShouldQuit()) {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        // Swap frames and Process Events
        pangolin::FinishFrame();
        // Update Physics and ModelGraph
        local_sim.StepForward();
        usleep( 1E6 / 60 );
      }
    } else {
      while (1) {
        local_sim.StepForward();
      }
    }
  }
  return 0;
}
