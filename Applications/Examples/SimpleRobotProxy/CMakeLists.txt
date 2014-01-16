cmake_minimum_required(VERSION 2.8)

find_package( Pangolin REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( Bullet REQUIRED )
find_package( TinyXML2 REQUIRED )

include_directories( ${CMAKE_SOURCE_DIR}/.. )
include_directories(${BULLET_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${SceneGraph_INCLUDE_DIRS})
include_directories(${TinyXML2_INCLUDE_DIRS})

link_libraries(${SceneGraph_LIBRARIES})
link_libraries(${Pangolin_LIBRARIES})
link_libraries(${BULLET_LIBRARIES})
link_libraries(${TinyXML2_LIBRARIES})
link_libraries(SimMessages) # common protocol buffers

link_directories( ${Pangolin_LIBRARY_DIRS} ) # this is bad form, libraries should specify full paths

add_executable( SimpleRobotProxy SimpleRobotProxy.cpp )

target_link_libraries( SimpleRobotProxy
                       SimMessages
                      )

