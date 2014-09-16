cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

find_package(SceneGraph 0.1 REQUIRED)
find_package(Pangolin)
find_package(Node REQUIRED)
find_package(URDFParser REQUIRED)
find_package(Calibu)
find_package(SimBA REQUIRED)
find_package(MINIGLOG REQUIRED)
find_package(CVARS REQUIRED)
find_package(Protobuf REQUIRED)
find_package(ASSIMP REQUIRED)

include_directories( ${Pangolin_INCLUDE_DIRS}
  ${SceneGraph_INCLUDE_DIRS}
  ${Node_INCLUDE_DIRS}
  ${Calibu_INCLUDE_DIRS}
  ${SimBA_INCLUDE_DIRS}
  ${MINIGLOG_INCLUDE_DIRS}
  ${CVARS_INCLUDE_DIRS}
  ${URDFParser_INCLUDE_DIRS}
  )

add_executable( LocalSim LocalSim.cpp LocalSim.h )

target_link_libraries( LocalSim
                       ${SceneGraph_LIBRARIES}
                       ${Node_LIBRARIES}
                       ${SimBA_LIBRARIES}
		       ${MINIGLOG_LIBRARIES}
		       ${PROTOBUF_LIBRARIES}
		       ${URDFParser_LIBRARIES}
		       ${ASSIMP_LIBRARIES}
		       )
