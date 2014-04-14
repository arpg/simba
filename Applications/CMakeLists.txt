add_subdirectory( LocalSim )
add_subdirectory( StateKeeper )
add_subdirectory( Examples )
if(${BUILD_SIM_PLANNER})
  add_subdirectory( SimPlanner )
endif()


