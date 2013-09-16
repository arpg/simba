cmake_minimum_required(VERSION 2.8)

find_package( Bullet REQUIRED )
include_directories(${BULLET_INCLUDE_DIRS})
link_libraries(${BULLET_LIBRARIES})
