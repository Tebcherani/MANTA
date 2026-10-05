cmake_minimum_required(VERSION 3.8)
project(whale_sim)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(manta_interfaces REQUIRED)
find_package(whale_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# -------------------------------------------------------------------------
# Whale simulation ROS node
# -------------------------------------------------------------------------

add_executable(
  whale_node
  src/whale_node.cpp
  src/whale_callbacks.cpp
  src/whale_interfaces.cpp
  src/whale_setup.cpp
)

# -------------------------------------------------------------------------
# Include paths for the existing Odyssey simulator code
# -------------------------------------------------------------------------

target_include_directories(
  whale_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(whale_node PUBLIC cxx_std_17)

# -------------------------------------------------------------------------
# ROS dependencies
# -------------------------------------------------------------------------

ament_target_dependencies(
  whale_node
  rclcpp
  manta_interfaces
  whale_interfaces
  std_msgs
)

# -------------------------------------------------------------------------
# Installation
# -------------------------------------------------------------------------

install(
  TARGETS whale_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()