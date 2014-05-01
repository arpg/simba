cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

find_package( Pangolin 0.1 QUIET )
find_package( Calibu REQUIRED )
#find_package( SceneGraph REQUIRED )
find_package( HAL 0.1 QUIET)
find_package( PbMsgs REQUIRED )
find_package( SimBA )
find_package( Node )
find_package( MINIGLOG )
#find_package( TinyXML2 REQUIRED )


include_directories( ${Pangolin_INCLUDE_DIRS})
#include_directories( ${SceneGraph_INCLUDE_DIRS})
include_directories( ${HAL_INCLUDE_DIRS})
include_directories( ${PbMsgs_INCLUDE_DIRS})
include_directories( ${MINIGLOG_INCLUDE_DIRS})
#include_directories( ${TinyXML2_INCLUDE_DIRS})
include_directories( ${SimBA_INCLUDE_DIRS})
include_directories( ${Calibu_INCLUDE_DIRS})
link_directories(${HAL_LINK_DIRS})

add_executable( TestCam TestCam.cpp )

#${SceneGraph_LIBRARIES}
#${TinyXML2_LIBRARIES}
target_link_libraries( TestCam
  ${Pangolin_LIBRARIES} 
  ${HAL_LIBRARIES}
  ${PbMsgs_LIBRARIES}
  ${Node_LIBRARIES}
  ${SimBA_LIBRARIES}
  ${Calibu_LIBRARIES}
  #{MINIGLOG_LIBRARIES}
  SimMessages
 )
