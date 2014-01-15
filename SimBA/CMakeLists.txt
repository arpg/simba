project( SimBA )
cmake_minimum_required(VERSION 2.4.3)

###################
# LINK TO OUR LIBRARIES
###################

find_package( Bullet REQUIRED )
find_package( Pangolin 0.1 QUIET )
find_package( SceneGraph 0.1 REQUIRED )
find_package( HAL 0.1 QUIET)
find_package( Calibu 0.1 REQUIRED )
find_package( Protobuf REQUIRED )
find_package( NODE REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( TinyXML2 REQUIRED )

# Library directories
include_directories( ${BULLET_INCLUDE_DIR})
include_directories( ${Pangolin_INCLUDE_DIRS})
include_directories( ${SceneGraph_INCLUDE_DIRS})
include_directories( ${HAL_INCLUDE_DIRS} )
include_directories( ${Calibu_INCLUDE_DIRS} )
include_directories( ${NODE_INCLUDE_DIRS})
include_directories( ${EIGEN3_INCLUDE_DIRS} )
include_directories( ${TinyXML2_INCLUDE_DIRS} )

# Source directories
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/Managers )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/ModelGraph )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/Network )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/SimDevices )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/SimRobots )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/URDFParser )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/Utils )
#The pb.h files are generated in build, so link there, too.
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# LINK ALL THE LIBRARIES!!!
link_libraries( ${BULLET_LIBRARIES} )
link_libraries( ${Pangolin_LIBRARIES} )
link_libraries( ${SceneGraph_LIBRARIES} )
link_libraries( ${Calibu_LIBRARIES} )
link_libraries( ${NODE_LIBRARIES} )
link_libraries( ${TinyXML2_LIBRARIES} )
link_libraries( ${EIGEN3_LIBRARIES} )
link_libraries( SimMessages )


################
# HEADER FILES FOR SIMBA LIB
################

list(APPEND MANAGERS_HDRS
  Managers/SimDeviceManager.h
  Managers/RobotsManager.h
  Managers/WorldManager.h
  Managers/NetworkManager.h
)

list(APPEND MODELGRAPH_HDRS
  ModelGraph/Body.h
  ModelGraph/Joint.h
  ModelGraph/Models.h
  ModelGraph/SE3.h
  ModelGraph/BulletModelGraphAgent.h
  ModelGraph/Link.h
  ModelGraph/PhyModelGraphAgent.h
  ModelGraph/Shape.h
  ModelGraph/ModelGraphBuilder.h
  ModelGraph/PhysicsEngine.h
  ModelGraph/GL_ShapeDrawer.h
  ModelGraph/Model.h
  ModelGraph/PhysicsEngineHelpers.h
  ModelGraph/GlutStuff.h
  ModelGraph/ModelNode.h
  ModelGraph/RenderClass.h
)

list(APPEND NETWORK_HDRS
  Network/Tools.h
  Network/WorldState.h
)

list(APPEND SIMDEVICES_HDRS
  SimDevices/SimDeviceInfo.h
  SimDevices/SimDevices.h
)

list(APPEND SIMDEVICES_CONTROLLER_HDRS
  SimDevices/Controller/CarController.h
  SimDevices/Controller/HingeController.h
  SimDevices/Controller/PoseController.h
  SimDevices/Controller/PIDControl.h
  SimDevices/Controller/SimpleAIController.h
  SimDevices/Controller/Controller.h
  SimDevices/Controller/PIDController.h
  SimDevices/Controller/SimpleController.h
)

list(APPEND SIMDEVICES_SENSOR_HDRS
  SimDevices/Sensor/SimGPS.h
  SimDevices/Sensor/SimLaser3D.h
  SimDevices/Sensor/SimVicon.h
  SimDevices/Sensor/SimCam.h
  SimDevices/Sensor/SimLaser2D.h
  SimDevices/Sensor/SimOdometry.h
)

list(APPEND SIMROBOTS_HDRS
  SimRobots/MobileModel.h
  SimRobots/SimRobot.h
)

list(APPEND URDFPARSER_HDRS
  URDFParser/RobotProxyURDFParser.h
  URDFParser/TinXMLTool.h
  URDFParser/GenURIFromURDF.h
  URDFParser/StateKeeperURDFParser.h
#  URDFParser/URDFParser.h
)

list(APPEND UTILS_HDRS
  Utils/ConvertName.h
  Utils/CVarHelpers.h
  Utils/GetPot
  Utils/MatrixTools.h
  Utils/SE3.h
)

list(APPEND SIMBA_HDRS ${MANAGERS_HDRS}           ${MODELGRAPH_HDRS}
                       ${NETWORK_HDRS}            ${NETWORK_HDRS}
                       ${SIMDEVICES_HDRS}         ${SIMDEVICES_CONTROLLER_HDRS}
                       ${SIMDEVICES_SENSOR_HDRS}  ${SIMROBOTS_HDRS}
                       ${URDFPARSER_HDRS}         ${UTILS_HDRS}
)

##########
# Let's list some SOURCE FILES, yo! What what.
##########
list(APPEND MANAGERS_SRCS
  Managers/RobotsManager.cpp
  Managers/NetworkManager.cpp
  Managers/SimDeviceManager.cpp
)
list(APPEND MODELGRAPH_SRCS)
list(APPEND NETWORK_SRCS)
list(APPEND SIMDEVICES_CONTROL_SRCS)
list(APPEND SIMDEVICES_SENSOR_SRCS)
list(APPEND SIMROBOTS_SRCS)
list(APPEND URDFPARSER_SRCS)
list(APPEND UTILS_SRCS)

list(APPEND SIMBA_SRCS ${MANAGERS_SRCS})


##################
# CREATE THE SIMBA LIBRARY
##################

add_library(${PROJECT_NAME} ${SIMBA_SRCS} ${SIMBA_HDRS})




