#------------------------------------------------------------------------------#
#                                                                              #
#                              COMPILER FLAGS                                  #
#                                                                              #
# CPPONLY flag removes mex-portions (includes and MEX iterface function) to    #
# allow C++ independent testing                                                #
#------------------------------------------------------------------------------#
# --- RELEASE --- #
# This is kind of verbose, due to the number of libraries this program has. 
PANGOLIN_DIR = /Users/Trystan/Code/rslam/rslam/build/CoreDev/Pangolin/
HAL_SRC_DIR = /Users/Trystan/Code/rslam/rslam/CoreDev/HAL/
HAL_DIR = /Users/Trystan/Code/rslam/build/CoreDev/HAL/
NODE_DIR = /Users/Trystan/Code/rslam/rslam/CoreDev/Node/
NODE_BUILD_DIR = /Users/Trystan/Code/rslam/build/CoreDev/Node/
PbMsgs_DIR = /Users/Trystan/Code/rslam/rslam/CoreDev/HAL/PbMsgs/
PbMsgs_BUILD_DIR = /Users/Trystan/Code/rslam/build/CoreDev/HAL/PbMsgs/
PROTOBUF_DIR = /usr/local/Cellar/protobuf/2.5.0/include/
PROTOBUF_LIB_DIR = /usr/local/Cellar/protobuf/2.5.0/lib/
MINIGLOG_BUILD_DIR = /Users/Trystan/Code/rslam/build/CoreDev/miniglog/
MINIGLOG_DIR = /Users/Trystan/Code/rslam/rslam/CoreDev/miniglog/
ZMQ_DIR = /usr/local/include/
BOOST_LIB_DIR = /usr/local/lib/
GENERAL_HDR = /usr/include/
CURRENT_DIR = /Users/Trystan/Code/simba_tests/MatlabInterface/

CXXFLAGS = -std=c++11 -O -I/Applications/MATLAB_R2013a.app/extern/include\
  -I$(HAL_SRC_DIR) -I$(HAL_DIR) -I$(NODE_DIR) -I$(NODE_BUILD_DIR)\
  -I$(PANGOLIN_DIR) -I$(PbMsgs_DIR) -I$(PbMsgs_BUILD_DIR)\
  -I$(PROTOBUF_DIR) -I$(ZMQ_DIR)\
  -I$(GENERAL_HDR) -I$(CURRENT_DIR) -I$(MINIGLOG_DIR)\
  -I$(PbMsgs_BUILD_DIR)BVP.pb.cc\
  -I$(NODE_BUILD_DIR)NodeMessages.pb.cc 

MXXFLAGS = -O -I/Applications/MATLAB_R2013a.app/extern/include\
  -I$(HAL_SRC_DIR) -I$(HAL_DIR) -I$(NODE_DIR) -I$(NODE_BUILD_DIR)\
  -I$(PANGOLIN_DIR) -I$(PbMsgs_DIR) -I$(PbMsgs_BUILD_DIR)\
  -I$(PROTOBUF_DIR) -I$(ZMQ_DIR)\
  -I$(GENERAL_HDR) -I$(CURRENT_DIR) -I$(MINIGLOG_DIR)\
  -I$(PbMsgs_BUILD_DIR)BVP.pb.cc\
  -I$(NODE_BUILD_DIR)NodeMessages.pb.cc 

CXXLIBS = -L$(PROTOBUF_LIB_DIR) -lprotobuf\
  -L$(NODE_BUILD_DIR) -lnode\
  -L$(MINIGLOG_BUILD_DIR) -lminiglog

MXXLIBS = -L$(PROTOBUF_LIB_DIR) -lprotobuf\
  -L$(NODE_BUILD_DIR) -lnode\
  -L$(MINIGLOG_BUILD_DIR) -lminiglog

# --- DEBUG   --- #
# CXXFLAGS = -j2 -O2 -Wall -fmessage-length=0 -D CPPONLY -D DEBUG

#------------------------------------------------------------------------------#
#                                                                              #
#                            ARCHITECTURE VARIABLES                            #
#                                                                              #
# add these to your user profile in order to specify your own.                 #
#------------------------------------------------------------------------------#

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
#                                                                              #
#                              OTHER VARIABLES                                 #
#                                                                              #
#------------------------------------------------------------------------------#
ARCHIVENAME = SimBAPlanner

#------------------------------------------------------------------------------#
#                                                                              #
#                              DEPENDENCY RULES                                # 
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
HDRS = class_handle.hpp node_wrapper.h BVP.pb.h

TARGET = node_mex 

BINTARGET = $(TARGET:%=%.bin)
MEXTARGET = $(TARGET:%=%.$(MEXEXT))
### MANUALLY REDUCED TARGETS
# TARGET = kdtree_build
# MEXTARGET = kdtree_build.$(MEXEXT)

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
#                                                                              #
#                   CREATE A DISTRIBUTION IN A ZIP FILE                        #
#                                                                              #
# move resources to a kdtree folder, tar them, then remove the temp directory  # 
# and its content completely                                                   #
#------------------------------------------------------------------------------#
#dist:   
#	rm -rf $(ARCHIVENAME)
#	mkdir $(ARCHIVENAME)
#	cp *.mexmaci64 $(ARCHIVENAME)
#	cp *.m $(ARCHIVENAME)
#	cp -L *.h $(ARCHIVENAME) #follow symlinks
#	cp *.cpp $(ARCHIVENAME)
#	cp CHANGES $(ARCHIVENAME)
#	cp TODO $(ARCHIVENAME)
#	cp README $(ARCHIVENAME)
#	cp Makefile $(ARCHIVENAME)
#	#tar -cvf $(ARCHIVENAME).tar.gz $(ARCHIVENAME)
#	zip -r -v ${ARCHIVENAME}.zip ${ARCHIVENAME}
#	rm -rf $(ARCHIVENAME)
#	e
