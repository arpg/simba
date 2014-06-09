#==============================================================================#
# NODE-MATLAB CONNECTION MAKEFILE
# -------------------------------
# This connection, by nature, relies on the MEX C++ interface designed for
# MATLAB. MEX is not user-friendly, so I've been particularly verbose when
# designing instructions for the makefile and project.
#==============================================================================#

#---------------------------------#
# 1. LIBRARIES
# - For MATLAB and all libraries, list both the source and the build directory
#   below, using the accepted convention that I made up:
#     PROGRAM_DIR = path/to/src
#     PROGRAM_BUILD_DIR = path/to/build
# - Modify CXXFLAGS, MXXFLAGS to incorporate all directories.
# - .cc files must be added directly to the main header file used. It seems
#   really hacky, but I couldn't find another way to successfully import
#   generated protobuf files. 
# If you get 'undefined symbol' errors during compilation, add
# the libraries that hold that information in MXXLIBS.
#---------------------------------#
MATLAB_DIR = /Applications/MATLAB_R2013a.app/extern/include/

HAL_DIR = /Users/Trystan/Code/rslam/rslam/CoreDev/HAL/
HAL_BUILD_DIR = /Users/Trynstan/Code/rslam/build/CoreDev/HAL/
NODE_DIR = /Users/Trystan/Code/rslam/rslam/CoreDev/Node/
NODE_BUILD_DIR = /Users/Trystan/Code/rslam/build/CoreDev/Node/
PANGOLIN_BUILD_DIR = /Users/Trystan/Code/rslam/rslam/build/CoreDev/Pangolin/
PbMsgs_DIR = /Users/Trystan/Code/rslam/rslam/CoreDev/HAL/PbMsgs/
PbMsgs_BUILD_DIR = /Users/Trystan/Code/rslam/build/CoreDev/HAL/PbMsgs/
PROTOBUF_DIR = /usr/local/Cellar/protobuf/2.5.0/include/
PROTOBUF_LIB_DIR = /usr/local/Cellar/protobuf/2.5.0/lib/
MINIGLOG_DIR = /Users/Trystan/Code/rslam/rslam/CoreDev/miniglog/
MINIGLOG_BUILD_DIR = /Users/Trystan/Code/rslam/build/CoreDev/miniglog/
ZMQ_DIR = /usr/local/include/
GENERAL_HDR = /usr/include/
PROJECT_DIR = /Users/Trystan/Code/simba/Applications/MatlabInterface/SimbaMaster/

CXXFLAGS = -std=c++11 -O -I$(MATLAB_DIR)\
  -I$(HAL_DIR) -I$(HAL_BUILD_DIR) -I$(NODE_DIR) -I$(NODE_BUILD_DIR)\
  -I$(PANGOLIN_BUILD_DIR) -I$(PbMsgs_DIR) -I$(PbMsgs_BUILD_DIR)\
  -I$(PROTOBUF_DIR) -I$(ZMQ_DIR)\
  -I$(GENERAL_HDR) -I$(PROJECT_DIR) -I$(MINIGLOG_DIR)\
  -I$(NODE_BUILD_DIR)NodeMessages.pb.cc 

MXXFLAGS = -O -I$(MATLAB_DIR)\
  -I$(HAL_DIR) -I$(HAL_BUILD_DIR) -I$(NODE_DIR) -I$(NODE_BUILD_DIR)\
  -I$(PANGOLIN_BUILD_DIR) -I$(PbMsgs_DIR) -I$(PbMsgs_BUILD_DIR)\
  -I$(PROTOBUF_DIR) -I$(ZMQ_DIR)\
  -I$(GENERAL_HDR) -I$(PROJECT_DIR) -I$(MINIGLOG_DIR)\
  -I$(NODE_BUILD_DIR)NodeMessages.pb.cc 

 CXXLIBS = -L$(PROTOBUF_LIB_DIR) -lprotobuf\
  -L$(NODE_BUILD_DIR) -lnode\
  -L$(MINIGLOG_BUILD_DIR) -lminiglog

MXXLIBS = -L$(PROTOBUF_LIB_DIR) -lprotobuf\
  -L$(NODE_BUILD_DIR) -lnode\
  -L$(MINIGLOG_BUILD_DIR) -lminiglog

#---------------------------------#
# 2. OUR HEADERS AND SOURCE FILE           
# There can be multiple headers, but only one source
# Notice that the PbMsgs header must be included here as well. 
#---------------------------------#

HDRS = class_handle.hpp SimbaMaster.h BVP.pb.h
TARGET = SimbaMaster_mex

#---------------------------------#
# 3. ARCHITECTURE VARIABLES
# This shouldn't have to be modified, but if needed,
# add these to your user profile in order to specify your own. 
#---------------------------------#

ifndef MXX
	ifeq ($(shell uname),Darwin)
		MXX = /Applications/MATLAB_R2013a.app/bin/mex
		MEXEXT = $(shell /Applications/MATLAB_R2013a.app/bin/mexext)
#MXXFLAGS = $(MXXFLAGS)
#-I/Applications/MATLAB_R2013a.app/extern/include
	endif
	ifeq ($(shell uname),Linux)
		MXX = /usr/local/matlab/bin/mex
		MEXEXT = $(shell /usr/local/matlab/bin/mexext) 
		MXXFLAGS = -I/usr/local/matlab/extern/include
	endif
else
	MEXEXT = $(shell MEXEXT) 
endif

#------------------------------------------------------------------------------#
# BUILDING THE SYSTEM                                                          #
# You shouldn't have to touch anything below this line.                        #
#------------------------------------------------------------------------------#

BINTARGET = $(TARGET:%=%.bin)
MEXTARGET = $(TARGET:%=%.$(MEXEXT))

all: $(BINTARGET) $(MEXTARGET)

debug: CXXFLAGS += -D DEBUG -g
debug: all

%.bin : %.cpp 
	$(CXX) -c $(CXXFLAGS) -o $@ $< $(CXXLIBS)

%.mexmaci64 : %.cpp 
	$(MXX) -cxx $(MXXFLAGS) -o $@ $< $(MXXLIBS)
clean:
	@rm -f $(OBJS) $(BINTARGET) $(MEXTARGET)
	@rm -rf  *.dSYM
	@rm -rf *.bin
	@echo "clean completed"

#------------------------------------------------------------------------------#
# MORE INFO ON THE BUILD SYSTEM (for the curious)                              # 
#                                                                              #
# USAGE OF RELEASE AND DEBUG MODE                                              #
# to compile a release file (without debug hooks and more efficient) just type #
# make. On the other hand, to enable debugging portions of the code (output)   #
# and add debugging links to the binary (needed for line by line execution)    #
# use :make debug"                                                             #
#                                                                              #
# EXPLANATION OF A RULE                                                        #
# anything (%) that terminates in .bin for which an explicit rule is not       #  
# available is made dependable on the file which has same name but .cpp        #
# extension. The compiler (CXX) with options (CXXFLAGS) is called on each of   #
# the elements that trigger the rule ($@, which is left side of ":") and       #
# produces an output with filename expressed by the "first" of elements from   #
# which it depends ($< or right side of ":")                                   #
#------------------------------------------------------------------------------#
