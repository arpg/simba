find_package( Protobuf REQUIRED )
find_package( NODE REQUIRED )

PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS SimMessages.proto )

# compile our messages into a little library
add_library( SimMessages ${PROTO_SRCS})

include_directories( ${PROTOBUF_INCLUDE_DIRS} )
include_directories( ${NODE_INCLUDE_DIRS}  ${CMAKE_SOURCE_DIR} )
target_link_libraries(SimMessages ${NODE_LIBRARIES} )
install(TARGETS SimMessages DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/PB_Headers EXPORT SimMessagesTargets)
install(EXPORT SimMessagesTargets DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
