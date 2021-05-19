cmake_minimum_required(VERSION 3.16.2)
project(rostron_sender)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rostron_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

## Boost
find_package(Boost 1.56 REQUIRED COMPONENTS system)

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
endif(Boost_FOUND)


# Protobuf
add_subdirectory(include/rostron_sender/proto)

# Create executable
SET(
  SRCS_SIM_SENDER
  src/sim_sender.cpp
  src/net/udp_sender.cpp
)

SET(
  HEADERS_SIM_SENDER
  include/rostron_sender/net/udp_sender.h
)

add_executable(sim_sender ${SRCS_SIM_SENDER} ${HEADERS_SIM_SENDER})
target_link_libraries(sim_sender ${Protobuf_LIBRARIES} NAELIC::Proto ${Boost_SYSTEM_LIBRARY})
ament_target_dependencies(sim_sender rclcpp rostron_interfaces rclcpp_action)


target_include_directories(sim_sender PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Protobuf_INCLUDE_DIRS}
)

install(TARGETS sim_sender
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(Boost Protobuf)
ament_export_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${Protobuf_LIBRARIES} NAELIC::Proto)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()