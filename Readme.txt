Project Sim, please contact luma@gwu.edu for support

How to run: (remember to modify path relative settings in URDF file, e.g. Robot.xml)

============================================================================================================================================
For Robot Code (Examples/), you can use any of the following command:

    (1) Run Robot Code directlly. This will open real devices described in Robot.xml URDF file
    ./TestCam -n RCamera -r /Users/malu/Code/Luma/Sim/urdf/TestCam.xml

    (2) Run Robot in Sim Mode, connect to Host
    ./TestCam -n LCamera -r /Users/malu/Code/Luma/Sim/urdf/TestCam.xml

    (4) Run Robot in Sim Mode connect to StateKeeper (not support by this version)
    SIM=SimWorld=/Users/malu/Code/Luma/Sim/urdf/World.xml@StateKeeper ./Robot /Users/malu/Code/Luma/Sim/urdf/Robot.xml

    (5) Run Robot in Sim Mode. Connect to (not support by this version)
    SIM=SimWorld=/Users/malu/Code/Luma/Sim/urdf/World.xml ./Robot /Users/malu/Code/Luma/Sim/urdf/Robot.xml


=============================================================================================================================================
For RobotProxy command line Example:
    (1), RobotProxy name is 'Proxy1', use 'Robot.xml' urdf file to init robot, 'World.xml' to init the world (mesh, light, ect) and run in WithoutNetwork mode
    -n Proxy1 -r /Users/malu/Code/Luma/Sim/urdf/Robot.xml -w /Users/malu/Code/Luma/Sim/urdf/World.xml -s WithoutNetwork

    (2), RobotProxy name is 'Proxy2', use 'Robot.xml' urdf file to init robot, 'World.xml' to init the world (mesh, light, ect) and run in with network mode (connect to StateKeeper which name is also 'Statekeeper')
    -n Proxy2 -r /Users/malu/Code/Luma/Sim/urdf/Robot.xml -w /Users/malu/Code/Luma/Sim/urdf/World.xml -s StateKeeper

    (3), RobotProxy name is 'Proxy1', use 'Robot.xml' urdf file to init robot, 'World.xml' to init the world (mesh, light, ect) and run in WithNetwork mode (for Robot Code to connect)
    -n Proxy1 -r /Users/malu/Code/Luma/Sim/urdf/Robot.xml -w /Users/malu/Code/Luma/Sim/urdf/World.xml -s WithoutStateKeeper


==============================================================================================================================================
For StateKeeper, Example:
    (1), StateKeeper name 'StateKeeper', init with 'World.xml' URDF file (init Robot in different position).
    -n StateKeeper -w /Users/malu/Code/Luma/Sim/urdf/World.xml



===============================================================================================================================================
If you want to use Sim with RPG(HAL), you may have the following error when build Node2Cam.

[ 94%] [ 96%] Building C object HAL/CMakeFiles/hal.dir/Camera/Drivers/UVC/libuvc/src/misc.c.o
Building CXX object HAL/CMakeFiles/hal.dir/IMU/Drivers/ProtoReader/ProtoReaderIMUDriver.cpp.o
[ 98%] make[2]: *** No rule to make target `HAL/Camera/Drivers/Node2Cam/libNode2Cam.dylib', needed by `HAL/libhal.dylib'.  Stop.
make[2]: *** Waiting for unfinished jobs....

You just need to type make -j to build it again and this error will be solved.

Be careful! Please build Sim with GCC 4.7!! Or you may have strange errors!!
