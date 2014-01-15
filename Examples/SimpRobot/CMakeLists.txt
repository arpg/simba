cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

find_package( Pangolin 0.1 QUIET )
find_package( SceneGraph REQUIRED )
find_package( Bullet REQUIRED )
find_package( HAL 0.1 QUIET)
find_package( TinyXML2 REQUIRED )
find_package( SimBA )

include_directories( ${NODE_INCLUDE_DIRS} )
include_directories( ${BULLET_INCLUDE_DIRS} )
include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories( ${SceneGraph_INCLUDE_DIRS} )
include_directories( ${HAL_INCLUDE_DIRS} )
include_directories( ${TinyXML2_INCLUDE_DIRS} )

add_executable( SimpRobot SimpRobot.cpp )

target_link_libraries( SimpRobot
  ${SceneGraph_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${BULLET_LIBRARIES}
  ${HAL_LIBRARIES}
  ${TinyXML2_LIBRARIES}
  ${SimBA_LIBRARIES}
  SimMessages
                       )
