cmake_minimum_required(VERSION 2.8)
project(StateKeeper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

find_package( NODE REQUIRED)
find_package( Eigen3 REQUIRED)
find_package( TinyXML2 REQUIRED )
find_package( Pangolin 0.1 QUIET )

include_directories( ${CMAKE_SOURCE_DIR}/.. )
include_directories( ${NODE_INCLUDE_DIRS} )
include_directories( ${EIGEN3_INCLUDE_DIRS} )
include_directories( ${TinyXML2_INCLUDE_DIRS} )
include_directories( ${Pangolin_INCLUDE_DIRS})

link_libraries( ${TinyXML2_LIBRARIES})
link_libraries( SimMessages) # common protocol buffers
link_libraries( ${Pangolin_LIBRARIES})

add_executable( StateKeeper StateKeeper.cpp )

target_link_libraries( StateKeeper ${NODE_LIBRARIES} )
