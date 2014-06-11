SimBA - Simulating 'Bots in Action
====================================

SimBA is a Robotics Simulator that hopes to make virtual physical simulation easy-to-use. Hardware interfaces through HAL (see below) make it an easy-to-use state keeper as well, working as an interface between the real and virtual world.

HOW TO BUILD
====================================

The below libraries are needed in order to install (provided in suggested order):
- Protobuf
- ZMQ - after installing this library, make sure that the .hpp file is also downloaded and added to your ZMQ directory (most likely at /usr/local/include). The .hpp can be found at https://github.com/zeromq/cppzmq/blob/master/zmq.hpp
- TinyXML2
- Bullet Physics
- Pangolin
- SceneGraph
- HAL (HAL and PbMsgs)
- Node
- Eigen 3
- Miniglog
- Calibu

CMake Options:
- BUILD CAR PLANNER: Builds the sections of SimBA that only work with the PlannerLib repo. Check it out from the RPG github, ssh://bminortx@robotics.gwu.edu/home/rpg/git/CarPlanner
- BUILD MATLAB INTERFACE: Makes the MATLAB interface files, PlannerMaster and SimbaMaster. *Before switching this option to ON, change the Makefiles included in their directories (./Applications/MatlabInterface/---Planner)*.

After these options are set, run cmake and make on SimBA. You are now ready to do some sweet simulation!

URDF FILES
====================================

SimBA relies on .xml files to describe the conditions of the World and the Robotic system that the user wants to control. There are a variety of examples provided in the code that allow the user to see what kinds of conditions SimBA can simulate. 

WORLDS
---------
Directory: ./urdf/Worlds/
- Basic: Just a flat plane
- Heightmap: Uses csv's for <X Y Z> data to generate a defined terrain
- Lost World: Imprts a .dae mesh file into SimBA and uses it as terrain
- MATLAB: Uses the MatlabInterface for Node (found in ./Applications/SimPlanner for now) to import a MATLAB-generated heightmap. This is a work in progress, so further documentation will be written in the future.

ROBOTS
---------
Directory: ./urdf/Robots/
- RaycastVehicle.xml: Describes the body, wheels, and action of a Bullet btRaycastVehicle. It can also import a .blend file for both the tires and the body of the car. 

APPLICATIONS
====================================

LocalSim (Main Demo)
---------
LocalSim is a wrapper for SimBA, and acts as our primary simulation tool. While running LocalSim, one can use Node and HAL in tandem to take simulated video, send commands to a vehicle, or track conditions using simulated GPS and IMU data (future versions, again). LocalSim requires several command line options to operate:
- *-n <SimName>*: The name of the simulator (important for HAL and Node interactions)
- *-r <Robot.xml>*: The .xml file being used to design the Robot. 
- *-w <World.xml>*: The .xml file being used to design the World.
- *-s <Statekeeper Option>*: What kind of network this simulation will have, if at all. There are a couple of options for this:
	1. WithoutNetwork: nothing is connecting to the LocalSim
	2. WithoutStateKeeper: HAL devices can control vehicles and sensors described in the Robot.xml file
	3. WithStateKeeper: LocalSim communicates with other LocalSims, all of which share a synchronization program between them called StateKeeper. This is a task for a future iteration of SimBA.
- *-debug*: Add this flag to the end to print logging info while LocalSim initializes.

As an example, from ./build:

	./Applications/LocalSim/LocalSim -n Ricky -r ../urdf/Robots/RaycastVehicle.xml -w ../urdf/Worlds/world_heightmap.xml -s WithoutStateKeeper

...starts a LocalSim named "Ricky", which runs the RaycastVehicle as described in its file, and the Heightmap data as described in its file. It also opens a Node connection, but does not attempt to interact with a StateKeeper. Adding -debug at the end of this command line would print the startup process. 

StateKeeper
---------
Work in progress...

MatlabInterface
---------

The MatlabInterface is actually a set of programs: SimbaMaster and PlannerMaster. SimbaMaster is a basic interaction tool with LocalSim (see Applications->LocalSim below), and has its own commands for building and testing RaycastVehicle performance in LocalSim.

PlannerMaster interacts directly with PlannerLib, and as such will only build if BUILD CAR PLANNER is set to ON. PlannerMaster takes commands found from PlannerLib and passes them to MATLAB, where they can then be transferred to LocalSim for plotting.

There will be an example .m file in the future demonstrating these programs in more detail, but they are currently still in development.

SimPlanner
---------
Work in progress...

Examples/KeyboardCarCommander
---------
Despite its name, KeyboardCarCommander does not use the keyboard at all; rather, it serves as an example of the HAL device <-> LocalSim connection. Running KeyboardCarCommander and LocalSim simultaneously creates a Node connection between them; the main loop in KCC then sends torque and steering commands to the RaycastVehicle (go slow, and turn to the left). It's easy to build the interactivity from here.

**N.B. - Naming.** This is a good place to introduce naming convention in SimBA. Names for devices are specified in multiple files, in case we might want to use more than one camera, or gather data from several different sources. Make sure to match the names of the camera and LocalSim across files. For example, RaycastVehicle.xml has a CarController named 'KeyboardCommander'. In the command line, we initialied our LocalSim with the name 'Ricky'. Hence, the main.cpp file of KCC inits a NodeCar device with the name=VehicleController and the sim=Ricky. If this is not correct, LocalSim will continue to run, but Node will not be able to control the car. 

Examples/TestCam
---------
With a LocalSim started and a camera initiated, TestCam allows the user to see through that camera. It gathers simulated video/photo data and sends it through Node to a separate window. It just serves as another example of the HAL<->LocalSim connection.

Examples/PathPlannerTest
---------
*Must have BUILD CAR PLANNER switched ON.* This program currently takes a start and goal configuration, along with a mesh, and uses them to calculate the optimum control policy to get from start to goal. The program then starts a node instance and passes the commands to LocalSim (assuming it's up and running already)

Examples/WaypointAddition
---------
WaypointAddition demonstrates the use of an RPC call to change the rendering while a LocalSim simulation is running. Running WaypointAddition adds a waypoint at the coordinate specified in the main.cpp. This methodology is handy when trying to add planner paths generated by PathPlannerTest. 


******************************

MAIN DEVELOPERS:
----------
* Brandon Minor
* Lu Ma
