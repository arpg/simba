project( SimPlanner )

cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMakeModules/")
set(MATLAB_INCLUDE_DIR "/Applications/MATLAB_R2013a.app/extern/include")
set(MATLAB_LIBRARIES "/Applications/MATLAB_R2013a.app/bin/maci64/libmex.dylib"
                     "/Applications/MATLAB_R2013a.app/bin/maci64/libmx.dylib"
                     "/Applications/MATLAB_R2013a.app/bin/maci64/libeng.dylib"
                     "/Applications/MATLAB_R2013a.app/bin/maci64/libmat.dylib")

find_package( Node REQUIRED )
find_package( MVL REQUIRED )
find_package( SimBA REQUIRED )
find_package( CarPlanner REQUIRED )
find_package( Calibu REQUIRED )
find_package( MINIGLOG REQUIRED )

include_directories(
  ${Node_INCLUDE_DIRS}
  ${SimBA_INCLUDE_DIRS}
  ${CAR_PLANNER_INCLUDE_DIRS}
  ${MVL_INCLUDE_DIRS}
  ${Calibu_INCLUDE_DIRS}
  ${MATLAB_INCLUDE_DIR}
  ${MINIGLOG_INCLUDE_DIRS})

add_executable( SimPlanner SimPlanner.cpp SimPlanner.h)

add_executable( MatlabMaster MatlabMaster.cpp MatlabMaster.h)

target_link_libraries( SimPlanner
                       ${Node_LIBRARIES}
                       ${SimBA_LIBRARIES}
                       ${CAR_PLANNER_LIBRARIES}
                       ${Calibu_LIBRARIES}
		       miniglog)

target_link_libraries( MatlabMaster
                       ${Node_LIBRARIES}
                       ${SimBA_LIBRARIES}
                       ${MATLAB_LIBRARIES}
		       miniglog)