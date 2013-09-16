cmake_minimum_required(VERSION 2.4.3)

include_directories( ${CMAKE_SOURCE_DIR}/src ${CMAKE_CURRENT_BINARY_DIR}) # for RobotGames/Messages.h
include_directories( ${CMAKE_BINARY_DIR}/src ) # for RobotGames/Messages.pb.h

add_subdirectory( Network ) # comment this if you don't want to use network
add_subdirectory( Device )
add_subdirectory( Robots )
add_subdirectory( URDFParser )
add_subdirectory( ModelGraph )
