project(WaypointAddition)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

find_package( HAL 0.1 QUIET )
find_package( Pangolin )
find_package( Node )
find_package( Calibu )
find_package( MINIGLOG )
find_package( SimBA )
include_directories( ${HAL_INCLUDE_DIRS} )
include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories( ${Node_INCLUDE_DIRS} )
include_directories( ${MINIGLOG_INCLUDE_DIRS} )
include_directories( ${Calibu_INCLUDE_DIRS})
include_directories( ${SimBA_INCLUDE_DIRS} )
link_directories(${HAL_LINK_DIRS})

add_executable( ${PROJECT_NAME} main.cpp )
target_link_libraries(
  ${PROJECT_NAME}
  ${Pangolin_LIBRARIES}
  ${HAL_LIBRARIES}
  ${Node_LIBRARIES}
  ${Calibu_LIBRARIES}
  ${MINIGLOG_LIBRARIES}
  ${SimBA_LIBRARIES}
  NodeMessages
)
