cmake_minimum_required(VERSION 2.8)

find_package( Eigen3 REQUIRED)
find_package( TinyXML2 REQUIRED )

include_directories( ${CMAKE_SOURCE_DIR}/.. )
include_directories( ${EIGEN3_INCLUDE_DIRS} )
include_directories( ${TinyXML2_INCLUDE_DIRS} )

link_libraries( ${EIGEN3_LIBRARIES})
link_libraries( ${TinyXML2_LIBRARIES})
