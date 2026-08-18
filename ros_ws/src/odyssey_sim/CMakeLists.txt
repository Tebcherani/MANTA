cmake_minimum_required(VERSION 3.8)
project(odyssey_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# -------------------------------------------------------------------------
# Odyssey ROS node
# -------------------------------------------------------------------------

add_executable(
  odyssey_node
  src/odyssey_node.cpp
)

# -------------------------------------------------------------------------
# Include paths for the existing Odyssey simulator code
# -------------------------------------------------------------------------

target_include_directories(
  odyssey_node
  PRIVATE
    /odyssey/include
    /odyssey/src/Common
    /odyssey/src/World
    /odyssey/src/Odyssey
    /odyssey/src/Odyssey/Vehicle
    /odyssey/src/Odyssey/Simulator
    /odyssey/src/Odyssey/Simulator/Faults
    /odyssey/src/Odyssey/Simulator/Sensors
)

# -------------------------------------------------------------------------
# ROS dependencies
# -------------------------------------------------------------------------

ament_target_dependencies(
  odyssey_node
  rclcpp
)

# -------------------------------------------------------------------------
# Existing Odyssey libraries
# -------------------------------------------------------------------------

target_link_libraries(
  odyssey_node
  /odyssey/libodyssey.a
  /odyssey/libsim_sensors.a
  /odyssey/libsim_common.a
  lapacke
  lapack
  blas
  m
)

# -------------------------------------------------------------------------
# Install executable into the ROS workspace
# -------------------------------------------------------------------------

install(
  TARGETS odyssey_node
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
