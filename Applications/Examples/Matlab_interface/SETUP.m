function [ new_class_ptr ] = SETUP(bool_create_ptr, num_sims)

PANGOLIN_DIR = '/Users/Trystan/Code/rslam/rslam/build/CoreDev/Pangolin/';
HAL_SRC_DIR = '/Users/Trystan/Code/rslam/rslam/CoreDev/HAL/';
HAL_DIR = '/Users/Trystan/Code/rslam/build/CoreDev/HAL/';

NODE_DIR = '/Users/Trystan/Code/rslam/rslam/CoreDev/HAL/Node/';
NODE_BUILD_DIR = '/Users/Trystan/Code/rslam/build/CoreDev/HAL/Node/';

PbMsgs_DIR = '/Users/Trystan/Code/rslam/rslam/CoreDev/HAL/PbMsgs/';
PbMsgs_BUILD_DIR = '/Users/Trystan/Code/rslam/build/CoreDev/HAL/PbMsgs/';

PROTOBUF_DIR = '/usr/local/Cellar/protobuf/2.5.0/include/';
PROTOBUF_LIB_DIR = '/usr/local/Cellar/protobuf/2.5.0/lib/';
ZMQ_DIR = '/usr/local/include/';

GENERAL_HDR = '/usr/include/';


SIMPLANNER_PATH = ...
  '/Users/Trystan/Code/simba/build/Applications/SimPlanner/SimPlanner';

%   [PbMsgs_BUILD_DIR 'BVP.pb.cc'], ...
%   [NODE_BUILD_DIR 'NodeMessages.pb.cc'],...

% There's some trickiness in the .pb.cc files that we have to watch out for. 

mex ('-c', '-g', ['-I' HAL_SRC_DIR], ['-I' HAL_DIR], ['-I' NODE_DIR], ['-I' NODE_BUILD_DIR], ...
  ['-I' PANGOLIN_DIR], ['-I' PbMsgs_DIR], ['-I' PbMsgs_BUILD_DIR], ...
  ['-I' PROTOBUF_DIR], ['-I' ZMQ_DIR], ['-I' GENERAL_HDR],'BVP.pb.h',...
  'node_mex.cpp',...
  ['-L' PROTOBUF_LIB_DIR], '-lprotobuf', ...
  ['-L' NODE_BUILD_DIR], '-lnode');
%   'NodeMessages.pb.h',...
disp('>Linking complete!');
disp('------------------');

new_class_ptr = 'NULL';

if bool_create_ptr == true,   
  new_class_ptr = SimBAPlanner(num_sims, SIMPLANNER_PATH);
end

end