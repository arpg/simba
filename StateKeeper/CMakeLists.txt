cmake_minimum_required(VERSION 2.8)
project(StateKeeper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()


add_executable( StateKeeper StateKeeper.cpp StateKeeper.h )
target_link_libraries( StateKeeper SimBA )
