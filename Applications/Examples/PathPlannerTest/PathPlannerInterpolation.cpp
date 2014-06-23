#include "PathPlannerTester.h"

/***********************************
 * THE MAIN LOOP
 * This interpolates a path from a given spline configuration
 **********************************/

int main(int argc, char** argv){
  // These are the two trajectories we're comparing:
  double sol1[] =
      {-0.2, 0.8, 4, -1, -0.4, 0.8, 0, 0, 0.93128, 0, 1.8626, 0,
       3.1484, 0.73721, 3.8947, 0.18027, 4.6411, -0.37668};
      // {0, 0.6, 5, -2, -0.4, 1, 0, 0, 1.0617, 0, 2.1234, 0,
      //  4.7835, 0.43888, 4.9113, -0.61508, 5.0392, -1.669};
  double sol2[] =
      {-0.2, 0.8, 4.5, -1, -0.4, 0.8, 0, 0, 1.031, 0, 2.062, 0,
       3.3344, 1.5103, 4.2327, 1.0042, 5.1309, 0.49807};
      // {0, 0.6, 5, 2, -0.4, 1, 0, 0, 1.077, 0, 2.1541, 0,
      //  3.016, 2.8388, 4.008, 2.4194, 5, 2};
  double goal_pts[] = {sol1[2], sol1[3], sol1[4], sol1[5],
                       sol2[2], sol2[3], sol2[4], sol2[5]};
  double solution[18];
  for (int ii = 0; ii < 18; ii++){
    solution[ii] = (sol1[ii] + sol2[ii]) / 2;
  }
  // Start and goal configurations
  // <x, y, theta, vel>
  std::vector<double> start = {0, 0, solution[0], solution[1]};
  std::vector<double> goal = {solution[2], solution[3],
                              solution[4], solution[5]};
  // Spline points
  Eigen::VectorXd x_values = Eigen::VectorXd(6);
  Eigen::VectorXd y_values = Eigen::VectorXd(6);
  x_values << solution[6], solution[8], solution[10],
      solution[12], solution[14], solution[16];
  y_values << solution[7], solution[9], solution[11],
      solution[13], solution[15], solution[17];

  // Boilerplate stuff
  std::unique_ptr<PathPlannerTester> sim(new PathPlannerTester());
  std::string name = "Sim0";
  if (argc == 2) {
    name = argv[1];
  }
  sim->sim_planner_name_ = name;
  std::string number = sim->GetNumber(sim->sim_planner_name_);
  std::shared_ptr<URDF_Parser> parser = std::make_shared<URDF_Parser>(0);
  XMLDocument world_xml;
  const string& world_urdf_path =
      "/Users/Trystan/Code/simba/urdf/Worlds/world_localplanner.xml";
  GetXMLdoc(world_urdf_path, world_xml);
  std::shared_ptr<HeightmapShape> heightmap_data(
      parser->GetMeshData(world_xml));
  sim->Init(heightmap_data, start, goal);

  // Interpolate the path with the bezier control points that we were
  // provided
  std::unique_ptr<pb::PlannerPolicyMsg> policy(new pb::PlannerPolicyMsg());
  Eigen::Vector6d spline_vec;
  spline_vec<<goal_pts[0], goal_pts[1], 0, 0, 0, goal_pts[2];
  sim->planner_gui_.AddSplinePoints(spline_vec, 1);
  spline_vec<<goal_pts[4], goal_pts[5], 0, 0, 0, goal_pts[6];
  sim->planner_gui_.AddSplinePoints(spline_vec, 1);
  sim->SampleTrajectory(policy.get(), x_values, y_values);

  // Drive our car in SimBA
  KeyCarController KeyCar("NodeCar:[name=VehicleController,sim=Ricky]//",
                          policy.get());
  KeyCar.ApplyCommands();
  LOG(INFO) << "Done with all of our shenanigans!";
}
