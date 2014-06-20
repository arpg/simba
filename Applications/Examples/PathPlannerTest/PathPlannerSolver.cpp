#include "PathPlannerTest.h"

/***********************************
 * THE MAIN LOOP
 * Solves the control policy between two configurations
 **********************************/

int main(int argc, char** argv){
  // Start and goal configurations
  // <x, y, theta, vel>
  std::vector<double> start = {0., 0., -.4, .6};
  std::vector<double> goal = {5., -2., -.4, .5};

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
  std::shared_ptr<HeightmapShape> heightmap_data(
      parser->GetMeshData(world_xml));
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
