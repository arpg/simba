SimBA - Simulating 'Bots in Action
====================================

How to run: (remember to modify path relative settings in URDF file, e.g. Robot.xml)

Running a Robot
-----------------
See apps/Examples/ for sample robot code.

1. Run Robot Code directly. This will open real devices described in Robot.xml URDF file

```
    ./TestCam -n RCamera -r /Users/malu/Code/Luma/Sim/urdf/TestCam.xml
```

2. Run Robot in Sim Mode, connect to Host

```
    ./TestCam -n LCamera -r /Users/malu/Code/Luma/Sim/urdf/TestCam.xml
```

4. Run Robot in Sim Mode connect to StateKeeper (not supported by this version)

```
    SIM=SimWorld=/Users/malu/Code/Luma/Sim/urdf/World.xml@StateKeeper ./Robot /Users/malu/Code/Luma/Sim/urdf/Robot.xml
```

5. Run Robot in Sim Mode. Connect to (not supported by this version)

```
    SIM=SimWorld=/Users/malu/Code/Luma/Sim/urdf/World.xml ./Robot /Users/malu/Code/Luma/Sim/urdf/Robot.xml
```


Run a RobotProxy					  
-------------------
1. RobotProxy name is 'Proxy1', use 'Robot.xml' urdf file to init robot, 'World.xml' to init the world (mesh, light, etc) and run in WithoutNetwork mode

```
    ./RobotProxy -n Proxy1 -r /Users/malu/Code/Luma/Sim/urdf/Robot.xml -w /Users/malu/Code/Luma/Sim/urdf/World.xml -s WithoutNetwork
```

2. RobotProxy name is 'Proxy2', use 'Robot.xml' urdf file to init robot, 'World.xml' to init the world (mesh, light, etc) and run in with network mode (connect to StateKeeper which name is also 'Statekeeper')

```
    ./RobotProxy -n Proxy2 -r /Users/malu/Code/Luma/Sim/urdf/Robot.xml -w /Users/malu/Code/Luma/Sim/urdf/World.xml -s StateKeeper
```

3. RobotProxy name is 'Proxy1', use 'Robot.xml' urdf file to init robot, 'World.xml' to init the world (mesh, light, etc) and run in WithNetwork mode (for Robot Code to connect)

```
    ./RobotProxy -n Proxy1 -r /Users/malu/Code/Luma/Sim/urdf/Robot.xml -w /Users/malu/Code/Luma/Sim/urdf/World.xml -s WithoutStateKeeper
```


Run the StateKeeper
--------------------
1. StateKeeper name 'StateKeeper', init with 'World.xml' URDF file (init Robot in different position).

```
    ./StateKeeper -n StateKeeper -w /Users/malu/Code/Luma/Sim/urdf/World.xml
```
