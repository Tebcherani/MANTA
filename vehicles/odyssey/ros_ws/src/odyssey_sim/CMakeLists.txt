cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0069)
  cmake_policy(SET CMP0069 NEW)
endif()
project(odyssey_sim)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(manta_interfaces REQUIRED)

# -------------------------------------------------------------------------
# Odyssey ROS node
# -------------------------------------------------------------------------

add_executable(
  odyssey_node
  src/odyssey_node.cpp
  src/odyssey_callbacks.cpp
  src/odyssey_interfaces.cpp
  src/odyssey_output.cpp
  src/odyssey_setup.cpp
)

# -------------------------------------------------------------------------
# Include paths for the existing Odyssey simulator code
# -------------------------------------------------------------------------

target_include_directories(
  odyssey_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /odyssey/vehicles/odyssey/include
    /odyssey/vehicles/odyssey/src/Common
    /odyssey/vehicles/odyssey/src/World
    /odyssey/vehicles/odyssey/src/Sensors
    /odyssey/vehicles/odyssey/src/Odyssey
    /odyssey/vehicles/odyssey/src/Odyssey/Vehicle
    /odyssey/vehicles/odyssey/src/Odyssey/Simulator
    /odyssey/vehicles/odyssey/src/Odyssey/Simulator/Faults
    /odyssey/vehicles/odyssey/src/Odyssey/Simulator/Sensors
)

# -------------------------------------------------------------------------
# ROS dependencies
# -------------------------------------------------------------------------

ament_target_dependencies(
  odyssey_node
  rclcpp
  std_msgs
  manta_interfaces
)

# -------------------------------------------------------------------------
# Existing Odyssey libraries
# -------------------------------------------------------------------------

target_link_libraries(
  odyssey_node
  /odyssey/vehicles/odyssey/lib/libodyssey.a
  /odyssey/vehicles/odyssey/lib/libsim_sensors.a
  /odyssey/vehicles/odyssey/lib/libsim_common.a
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
