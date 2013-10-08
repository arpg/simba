cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

find_package( Pangolin 0.1 QUIET )
find_package( SceneGraph 0.1 REQUIRED )
find_package( Bullet 0.1 REQUIRED )
find_package( TinyXML2 REQUIRED )
find_package( HAL 0.1 QUIET)
find_package( Calibu 0.1 REQUIRED )
find_package( NODE REQUIRED)


include_directories( ${CMAKE_SOURCE_DIR}/.. )
include_directories( ${BULLET_INCLUDE_DIRS})
include_directories( ${Pangolin_INCLUDE_DIRS})
include_directories( ${SceneGraph_INCLUDE_DIRS})
include_directories( ${TinyXML2_INCLUDE_DIRS})
include_directories( ${HAL_INCLUDE_DIRS} )
include_directories( ${Calibu_INCLUDE_DIRS} )
include_directories( ${NODE_INCLUDE_DIRS})

link_directories( ${Pangolin_LIBRARY_DIRS} ) # this is bad form, libraries should specify full paths
link_directories( ${Calibu_LIBRARY_DIRS} )

add_executable( RobotProxy RobotProxy.cpp )

target_link_libraries( RobotProxy
                       ${NODE_LIBRARIES}
                       ${SceneGraph_LIBRARIES}
                       ${Pangolin_LIBRARIES}
                       ${TinyXML2_LIBRARIES}
                       ${Calibu_LIBRARIES}
                       ${BULLET_LIBRARIES}
                       SimMessages
                      )
