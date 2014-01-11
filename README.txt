====================================
SimBA - Simulating 'Bots in Action
====================================

SimBA is a Robotics Simulator that hopes to make virtual physical simulation easy-to-use. Hardware interfaces through HAL (see below) make it an easy-to-use state keeper as well, working as an interface between the real and virtual world. 

====================================
HOW TO BUILD
====================================
1. Compile the above libraries in the order suggested:
>>	Protobuf
>>	ZMQ - after installing this library, make sure that the .hpp file is also downloaded and added to your ZMQ directory (most likely at /usr/local/include). The .hpp can be found at _________________
>>	TinyXML2
>>	HAL - build through CMake. Turn BUILD_NODE to ON, in order to install the required Node driver. If you want to test the Node driver, turn Node2Cam driver to ON in the Cmake.
>>	Pangolin
>>	SceneGraph

2. Run cmake and make on SimBA. 

3. You are now ready to test!

====================================
HOW TO RUN
====================================

NB - Remember to modify path relative settings in the URDF files used, e.g. Robot.xml. If you do not use mesh, just change camera model file path in .xml file.

Make sure that your World.xml file is set to SIM in your local environment, if you're using a Simulation. You can do this by running the following: 
>>	SIM=SimWorld=[Path to world file]/World.xml
There are some modifications to this line that will be covered below. 

===================
(1) Run a RobotProxy
===================
The RobotProxy object creates a network over which data about the robot and the world can be communicated. There are three slightly different ways to run a RobotProxy. 

1. The RobotProxy name set in this example is 'Proxy1'. Use the 'Robot.xml' URDF file to initialize the robot, and the 'World.xml' to initialize the world environment (mesh, light, etc). We'll run this example in WithoutNetwork mode:

>>	./RobotProxy -n Proxy1 -r /Users/malu/Code/Luma/simba/urdf/Robot.xml -w /Users/malu/Code/Luma/simba/urdf/World.xml -s WithoutNetwork

2. This RobotProxy name is 'Proxy2'. Use 'Robot.xml' URDF file to initialize the robot, the 'World.xml' to initialize the world (mesh, light, etc). Now we'll run in WithNetwork mode; to do this, we connect to a StateKeeper object (conveniently named 'Statekeeper'):

>>	./RobotProxy -n Proxy2 -r /Users/malu/Code/Luma/simba/urdf/Robot.xml -w /Users/malu/Code/Luma/simba/urdf/World.xml -s StateKeeper

Look to section [(3) Run a StateKeeper] for more info.

3. The RobotProxy name is 'Proxy1'. Initialize the robot and world parameters as we did in (1) and (2), and run in WithNetwork mode (for Robot Code to connect), but without a StateKeeper. 

>>	./RobotProxy -n Proxy1 -r /Users/malu/Code/Luma/simba/urdf/Robot.xml -w /Users/malu/Code/Luma/simba/urdf/World.xml -s WithoutStateKeeper

===================
(2) Running a Robot
===================
There are several ways to run a Robot. See apps/Examples/ for sample robot code:

1. Run Robot Code directly. This will open real devices described in the Robot.xml URDF file.

>>	./TestCam -n RCamera -r /Users/malu/Code/Luma/simba/urdf/TestCam.xml

2. Run Robot in Sim Mode, and connect to the Host.

>>	./TestCam -n LCamera -r /Users/malu/Code/Luma/simba/urdf/TestCam.xml

Planned modes (not currently supported):
3. Run Robot in Sim Mode connect to StateKeeper

>>	SIM=SimWorld=/Users/malu/Code/Luma/simba/urdf/World.xml@StateKeeper ./Robot /Users/malu/Code/Luma/simba/urdf/Robot.xml

4. Run Robot in Sim Mode. Connect to (not supported by this version)

>>	SIM=SimWorld=/Users/malu/Code/Luma/simba/urdf/World.xml ./Robot /Users/malu/Code/Luma/simba/urdf/Robot.xml

===================
(3) Run a StateKeeper
===================
As its name suggests, the StateKeeper object records the sensor info from our robot over time. In this example, we name it 'StateKeeper', and initialize our world with the 'World.xml' URDF file. You'll notice that the Robot object has not been initialized; this is done separately.

>>	./StateKeeper -n StateKeeper -w /Users/malu/Code/Luma/simba/urdf/World.xml


