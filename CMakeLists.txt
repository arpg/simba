project( Simba )
cmake_minimum_required( VERSION 2.8 )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -fPIC")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

add_definitions(-DBT_USE_DOUBLE_PRECISION)
add_definitions(-DBT_NO_PROFILE)

# Add to module path, so we can find our cmake modules
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/.)
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

set( BUILD_CAR_PLANNER false CACHE BOOL force )
set( BUILD_MATLAB_INTERFACE false CACHE BOOL force )

#Create everything else.
add_subdirectory( SimBA )
add_subdirectory( Applications )
