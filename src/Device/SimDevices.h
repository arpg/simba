// the following sim device sim data from sim world (e.g. RobotProxy)

#include <iostream>
#include <vector>
#include <Eigen/Eigen>             // for vector maths
#include <Mvlpp/Mvl.h>


#include <SceneGraph/SceneGraph.h> // for open GL scene graph
#include <SceneGraph/SimCam.h>     // for sim cam
#include <ModelGraph/PhysicsClass.h>

// sensors
#include <Device/Sensor/SimCam.h>
#include <Device/Sensor/SimGPS.h>
#include <Device/Sensor/SimVicon.h>
#include <Device/Sensor/SimLaser2D.h>
#include <Device/Sensor/SimLaser3D.h>
#include <Device/Sensor/SimOdometry.h>

// controllers
#include <Device/Controller/PIDController.h>
#include <Device/Controller/SimpleController.h>
#include <Device/Controller/CarController.h>


using namespace SceneGraph;
