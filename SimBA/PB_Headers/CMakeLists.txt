find_package( Protobuf REQUIRED )
find_package( Node REQUIRED )

PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS
  SimMessages.proto
  BVP.proto
  RenderShapes.proto)

# compile our messages into a little library
add_library( NodeMessages ${PROTO_SRCS})

include_directories( ${PROTOBUF_INCLUDE_DIRS} )
include_directories( ${Node_INCLUDE_DIRS}  ${CMAKE_SOURCE_DIR} )
target_link_libraries(NodeMessages ${Node_LIBRARIES} )
install(TARGETS NodeMessages DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/PB_Headers EXPORT NodeMessagesTargets)
install(EXPORT NodeMessagesTargets DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
