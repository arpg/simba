project( PathPlannerTest )

cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMakeModules/")

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
  ${MINIGLOG_INCLUDE_DIRS})

add_executable( PathPlannerSolver PathPlannerSolver.cpp
  PathPlannerTest.cpp
  PlannerGui.cpp
  GLBulletDebugDrawer.cpp
  PathPlannerTest.h
  PlannerGui.h
  GLBulletDebugDrawer.h
  ClickHandler.h)
		     
target_link_libraries( PathPlannerSolver
  ${Node_LIBRARIES}
  ${SimBA_LIBRARIES}
  ${CAR_PLANNER_LIBRARIES}
  ${MVL_LIBRARIES}
  ${Calibu_LIBRARIES})

add_executable( PathPlannerInterpolation PathPlannerInterpolation.cpp
  PathPlannerTest.cpp
  PlannerGui.cpp
  GLBulletDebugDrawer.cpp
  PathPlannerTest.h
  PlannerGui.h
  GLBulletDebugDrawer.h
  ClickHandler.h)

target_link_libraries( PathPlannerInterpolation
  ${Node_LIBRARIES}
  ${SimBA_LIBRARIES}
  ${CAR_PLANNER_LIBRARIES}
  ${MVL_LIBRARIES}
  ${Calibu_LIBRARIES})