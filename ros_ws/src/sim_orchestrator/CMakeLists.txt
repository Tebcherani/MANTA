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
    /odyssey/vehicles/odyssey/include
    /odyssey/vehicles/odyssey/src/Common
    /odyssey/vehicles/odyssey/src/World
    /odyssey/vehicles/odyssey/src/Simulation
    /odyssey/vehicles/odyssey/src/Sensors
    /odyssey/vehicles/odyssey/src/Odyssey
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
# Existing Odyssey libraries
# -------------------------------------------------------------------------

target_link_libraries(
  sim_orchestrator_node
  "-Wl,--start-group"
  /odyssey/vehicles/odyssey/lib/libsimulation.a
  /odyssey/vehicles/odyssey/lib/libworld_sim.a
  /odyssey/vehicles/odyssey/lib/libsim_common.a
  /odyssey/vehicles/odyssey/lib/libsim_sensors.a
  /odyssey/vehicles/odyssey/lib/libodyssey.a
  "-Wl,--end-group"
  lapacke
  lapack
  blas
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
