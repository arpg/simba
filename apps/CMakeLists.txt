cmake_minimum_required(VERSION 2.8)

include_directories( ${CMAKE_SOURCE_DIR}/src ${CMAKE_CURRENT_BINARY_DIR}) # for RobotGames/Messages.h
include_directories( ${CMAKE_BINARY_DIR}/src ) # for RobotGames/Messages.pb.h

add_subdirectory( RobotProxy )
add_subdirectory( Examples )
add_subdirectory( StateKeeper) # comment this if you don't want to use StateKeeper


