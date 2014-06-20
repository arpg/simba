#include "PathPlannerTest.h"

/***********************************
 * THE MAIN LOOP
 * This interpolates a path from a given spline configuration
 **********************************/

int main(int argc, char** argv){
  // Start and goal configurations
  // <x, y, theta, vel>
  std::vector<double> start;
  std::vector<double> goal;
  // Old school like the old school
  double solution[] =
      {-0.4, 0.6, 5, -2, -0.4, 0.5, 0, 0, 1.0699, 0, 2.1397, 0, 3.1377,
       -0.68632, 4.1671, -0.97782, 5.1965, -1.2693};
  start.push_back(0);
  start.push_back(0);
  start.push_back(solution[0]);
  start.push_back(solution[1]);
  goal.push_back(solution[2]);
  goal.push_back(solution[3]);
  goal.push_back(solution[4]);
  goal.push_back(solution[5]);
  // Spline points
  Eigen::VectorXd x_values = Eigen::VectorXd(6);
  Eigen::VectorXd y_values = Eigen::VectorXd(6);
  x_values<<solution[6], solution[8], solution[10],
      solution[12], solution[14], solution[16];
  y_values<<solution[7], solution[9], solution[11],
      solution[13], solution[15], solution[17];

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


  // Interpolate the path with the bezier control points that we were
  // provided
  pb::PlannerPolicyMsg* policy = new pb::PlannerPolicyMsg();
  sim->SampleTrajectory(policy, x_values, y_values);

  // Drive our car in SimBA
  KeyCarController KeyCar("NodeCar:[name=VehicleController,sim=Ricky]//",
                          policy);
  KeyCar.ApplyCommands();
  LOG(INFO) << "Done with all of our shenanigans!";
}
