add_subdirectory( LocalSim )
add_subdirectory( StateKeeper )
add_subdirectory( Examples )
if(BUILD_SIMPLANNER)
  add_subdirectory( SimPlanner )
endif()


