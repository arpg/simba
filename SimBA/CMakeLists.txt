project( SimBA )
cmake_minimum_required(VERSION 2.4.3)

set(LIBRARY_NAME ${PROJECT_NAME})

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

#######################################################

set(USER_INC ${BULLET_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${SceneGraph_INCLUDE_DIRS}
  ${HAL_INCLUDE_DIRS}
  ${Calibu_INCLUDE_DIRS}
  ${NODE_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${TinyXML2_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR} )

# Library directories
include_directories( ${USER_INC} )


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
  URDFParser/LocalSimURDFParser.h
  URDFParser/TinyXMLTool.h
  URDFParser/GenURIFromURDF.h
  URDFParser/StateKeeperURDFParser.h
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
                       ${PB_HEADERS}
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

# LINK ALL THE LIBRARIES!!!
target_link_libraries( ${PROJECT_NAME}
  ${BULLET_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${SceneGraph_LIBRARIES}
  ${Calibu_LIBRARIES}
  ${NODE_LIBRARIES}
  ${TinyXML2_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  SimMessages
)


#######################################################
#######################################################
#######################################################
## Create configure file for inclusion in library

CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h"
)



#######################################################

# This relative path allows installed files to be relocatable.
set( CMAKECONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}" )
file( RELATIVE_PATH REL_INCLUDE_DIR
    "${CMAKE_INSTALL_PREFIX}/${CMAKECONFIG_INSTALL_DIR}"
    "${CMAKE_INSTALL_PREFIX}/include" )

# Export library for easy inclusion from other cmake projects.
file( REMOVE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake" )
export( TARGETS ${LIBRARY_NAME}
        APPEND FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake" )

# Version information
configure_file("${PROJECT_NAME}ConfigVersion.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)

# Build tree config
set( EXPORT_LIB_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}" )
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY IMMEDIATE )

# Install tree config
#set( EXPORT_LIB_INC_DIR "\${${PROJECT_NAME}_CMAKE_DIR}/${REL_INCLUDE_DIR}" )
#configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in"
#    "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY )

# Add package to CMake package registery for use from the build tree
option( EXPORT_${PROJECT_NAME}
  "Should the ${PROJECT_NAME} package be exported for use by other software" ON )

if( EXPORT_${PROJECT_NAME} )
  export( PACKAGE ${PROJECT_NAME} )
endif()

#######################################################
## Install headers / targets

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config.h"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
)
install(FILES ${HEADERS}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
)

install(TARGETS ${LIBRARY_NAME}
  EXPORT "${PROJECT_NAME}Targets"
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

#######################################################
## Install CMake config

INSTALL(
    FILES "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKECONFIG_INSTALL_DIR} )

install( EXPORT "${PROJECT_NAME}Targets" DESTINATION ${CMAKECONFIG_INSTALL_DIR} )
