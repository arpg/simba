// the following sim device sim data from sim world (e.g. RobotProxy)

#include <iostream>
#include <vector>
#include <Eigen/Eigen>             // for vector maths

#include <SceneGraph/SceneGraph.h> // for open GL scene graph
#include <SceneGraph/SimCam.h>     // for sim cam
#include <ModelGraph/PhysicsEngine.h>

// sensors
#include <SimDevices/Sensor/SimCam.h>
#include <SimDevices/Sensor/SimGPS.h>
#include <SimDevices/Sensor/SimVicon.h>
#include <SimDevices/Sensor/SimLaser2D.h>
#include <SimDevices/Sensor/SimLaser3D.h>
#include <SimDevices/Sensor/SimOdometry.h>

// controllers
#include <SimDevices/Controller/PIDController.h>
#include <SimDevices/Controller/SimpleController.h>
#include <SimDevices/Controller/CarController.h>


using namespace SceneGraph;
