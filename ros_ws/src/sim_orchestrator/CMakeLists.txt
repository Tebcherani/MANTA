cmake_minimum_required(VERSION 3.8)
project(sim_orchestrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# -------------------------------------------------------------------------
# Simulator orchestrator ROS node
# -------------------------------------------------------------------------

add_executable(
  sim_orchestrator_node
  src/sim_orchestrator_node.cpp
)

# -------------------------------------------------------------------------
# Include paths for the existing simulator orchestrator code
# -------------------------------------------------------------------------

target_include_directories(
  sim_orchestrator_node
  PRIVATE
    /odyssey/include
    /odyssey/src/Application
    /odyssey/src/Common
    /odyssey/src/World
    /odyssey/src/Simulation
)

# -------------------------------------------------------------------------
# ROS dependencies
# -------------------------------------------------------------------------

ament_target_dependencies(
  sim_orchestrator_node
  rclcpp
  std_msgs
)

# -------------------------------------------------------------------------
# Install executable into the ROS workspace
# -------------------------------------------------------------------------

install(
  TARGETS sim_orchestrator_node
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------------------------------------------------
# Tests / linting
# -------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
