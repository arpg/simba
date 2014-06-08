add_subdirectory( LocalSim )
add_subdirectory( StateKeeper )
add_subdirectory( Examples )
if(${BUILD_CAR_PLANNER})
  add_subdirectory( SimPlanner )
endif()


