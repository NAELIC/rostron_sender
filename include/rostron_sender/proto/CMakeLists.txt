cmake_minimum_required(VERSION 3.16.2)
find_package(Protobuf REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
  simulation/ssl_simulation_robot_control.proto
)

add_library(sslproto
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

add_library(NAELIC::Proto ALIAS sslproto)

target_include_directories(sslproto PUBLIC
    ${Protobuf__INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(sslproto PUBLIC ${Protobuf_LIBRARIES})
