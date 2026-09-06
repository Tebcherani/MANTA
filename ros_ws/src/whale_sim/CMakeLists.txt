cmake_minimum_required(VERSION 3.8)
project(whale_sim)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(manta_interfaces REQUIRED)

add_executable(whale_node src/whale_node.cpp)

ament_target_dependencies(
  whale_node
  rclcpp
  manta_interfaces
)

install(
  TARGETS whale_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()