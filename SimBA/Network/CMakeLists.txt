find_package( Protobuf REQUIRED )
find_package( Node REQUIRED )

PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS SimMessages.proto )

# compile our messages into a little library
add_library( SimMessages ${PROTO_SRCS})

include_directories( ${PROTOBUF_INCLUDE_DIRS} )
include_directories( ${Node_INCLUDE_DIRS}  ${CMAKE_SOURCE_DIR} )
target_link_libraries(SimMessages ${Node_LIBRARIES} )


