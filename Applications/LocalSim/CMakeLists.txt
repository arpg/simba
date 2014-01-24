cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

find_package( SceneGraph 0.1 REQUIRED )
find_package( Pangolin )
find_package( NODE REQUIRED )
find_package( Calibu )
find_package( SimBA REQUIRED )

include_directories( ${Pangolin_INCLUDE_DIRS}
  ${SceneGraph_INCLUDE_DIRS}
  ${NODE_INCLUDE_DIRS}
  ${Calibu_INCLUDE_DIRS}
  ${SimBA_INCLUDE_DIRS} )

add_executable( LocalSim LocalSim.cpp
                           LocalSim.h )

target_link_libraries( LocalSim
                       ${SceneGraph_LIBRARIES}
                       ${NODE_LIBRARIES}
                       ${SimBA_LIBRARIES}
                       SimMessages )
