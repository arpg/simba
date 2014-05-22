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

Run cmake and make on SimBA. You are now ready to test!

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
  > WithoutNetwork: nothing is connecting to the LocalSim
  > WithoutStateKeeper: HAL devices can control vehicles and sensors described in the Robot.xml file
  > WithStateKeeper: LocalSim communicates with other LocalSims, all of which share a synchronization program between them called StateKeeper. This is a task for a future iteration of SimBA. 

As an example, from ./build:
	./Applications/LocalSim/LocalSim -n Ricky -r ../urdf/Robots/RaycastVehicle.xml -w ../urdf/Worlds/world_heightmap.xml -s WithoutStateKeeper
...starts a LocalSim named "Ricky", which runs the RaycastVehicle as described in its file, and the Heightmap data as described in its file. It also opens a Node connection, but does not attempt to interact with a StateKeeper. 

StateKeeper
---------
Work in progress...

Examples/KeyboardCarCommander
---------
Despite its name, KeyboardCarCommander does not use the keyboard at all; rather, it serves as an example of the HAL device <-> LocalSim connection. Running KeyboardCarCommander and LocalSim simultaneously creates a Node connection between them; the main loop in KCC then sends torque and steering commands to the RaycastVehicle (go slow, and turn to the left). It's easy to build the interactivity from here.

Examples/TestCam
---------
With a LocalSim started and a camera initiated, TestCam allows the user to see through that camera. It gathers simulated video/photo data and sends it through Node to a separate window. It just serves as another example of the HAL<->LocalSim connection.

Examples/Matlab_interface
---------
Matlab_interface creates a Node connection between MATLAB and SimBA usig a MEX wrapper. It's a tricky beast, solely due to the progreamming restrictions brought on by MATLAB... but it does well. The MATLAB World above starts the Node connection on the LocalSim side in order to grab heightmap data from the MATLAB side.

SimPlanner
---------
Work in progress...

******************************

MAIN DEVELOPERS:
----------
* Brandon Minor
* Lu Ma
