#include <iostream>

#include <pangolin/pangolin.h>
#include <Node/Node.h>
#include <PbMsgs/SceneGraphShapes.pb.h>

/****************
 * MAIN LOOP
 * All this program does is sets up a SceneGraphMsg full of Waypoints, and
 * adds them to an already-running LocalSim.
 * It's just meant as a proof of concept.
 * **************/

int main(){
  node::node node_;
  node_.set_verbosity(0);
  node_.init("Waypoint");
  pb::RegisterRenderReqMsg req;
  pb::RegisterRenderRepMsg rep;
  pb::SceneGraphMsg graph;
  // Set up our waypoint
  pb::WaypointMsg waypoint;
  waypoint.set_name("first");
  waypoint.set_velocity(5);
  double* start_point = new double[6];
  start_point[0] = 5;
  start_point[1] = 5;
  start_point[2] = 5;
  start_point[3] = 0;
  start_point[4] = 0;
  start_point[5] = 0;
  for (int ii = 0; ii < 6; ii++) {
    waypoint.add_pose(start_point[ii]);
  }
  graph.set_allocated_waypoint(&waypoint);
  req.set_allocated_new_objects(&graph);
  int nTries = 0;
  // Send the damn thing
  while (nTries < 10000 &&
         node_.call_rpc("Ricky", "AddRenderObject",
                        req, rep) == false) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    nTries++;
  }
}
