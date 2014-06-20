#include "PathPlannerTest.h"

/***********************************
 * THE MAIN LOOP
 * This is a test, so we're only going to do ONE path through
 * the simplest type of a terrain, the flat plane. Get the commands
 * back from this simulation, and start a separate Node process
 * that sends these commands to LocalSim in SimBA.
 **********************************/

int main(int argc, char** argv){
  // Start and goal configurations
  // <x, y, theta, vel>
  std::vector<double> start;
  std::vector<double> goal;
  start.push_back(0);
  start.push_back(0);
  start.push_back(-.4);
  start.push_back(.6);
  goal.push_back(5);
  goal.push_back(-2);
  goal.push_back(-.4);
  goal.push_back(.5);

  // Boilerplate stuff
  PathPlannerTest* sim = new PathPlannerTest();
  std::string name = "Sim0";
  if (argc == 2) {
    name = argv[1];
  }
  sim->sim_planner_name_ = name;
  std::string number = sim->GetNumber(sim->sim_planner_name_);
  URDF_Parser* parser = new URDF_Parser(0);
  XMLDocument world_xml;
  const string& world_urdf_path =
      "/Users/Trystan/Code/simba/urdf/Worlds/world_localplanner.xml";
  GetXMLdoc(world_urdf_path, world_xml);
  HeightmapShape* heightmap_data = parser->GetMeshData(world_xml);
  sim->Init(heightmap_data, start, goal);


  // Solve for the path using PlannerLib
  pb::PlannerPolicyMsg* policy = new pb::PlannerPolicyMsg();
  sim->SolveTrajectory(policy);


  // Drive our car in SimBA
  KeyCarController KeyCar("NodeCar:[name=VehicleController,sim=Ricky]//",
                          policy);
  KeyCar.ApplyCommands();
  LOG(INFO) << "Done with all of our shenanigans!";
}
