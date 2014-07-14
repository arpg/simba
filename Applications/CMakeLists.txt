add_subdirectory(LocalSim)
# add_subdirectory( StateKeeper )  ...for another time.
add_subdirectory(Examples)
if(${BUILD_MATLAB_INTERFACE})
  add_subdirectory(MatlabInterface)
endif()



