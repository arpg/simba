cmake_minimum_required( VERSION 2.8 )

project( Sim )

# Add to module path, so we can find our cmake modules
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/.)

# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

add_subdirectory( apps )
add_subdirectory( src )
