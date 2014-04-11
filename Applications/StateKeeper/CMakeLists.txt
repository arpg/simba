cmake_minimum_required(VERSION 2.8)
project(StateKeeper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

#find_package( SceneGraph 0.1 REQUIRED )
#find_package( Pangolin )
find_package( Node REQUIRED )
find_package( Calibu )
find_package( SimBA REQUIRED )

include_directories(
#  ${Pangolin_INCLUDE_DIRS}
#  ${SceneGraph_INCLUDE_DIRS}
  ${NODE_INCLUDE_DIRS}
  ${Calibu_INCLUDE_DIRS}
  ${SimBA_INCLUDE_DIRS} )

add_executable( StateKeeper StateKeeper.cpp
  StateKeeper.h )

target_link_libraries( StateKeeper
                       ${NODE_LIBRARIES}
                       ${SimBA_LIBRARIES})
