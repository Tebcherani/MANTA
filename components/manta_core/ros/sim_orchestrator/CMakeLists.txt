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
find_package(acoustics_interfaces REQUIRED)
find_package(manta_interfaces REQUIRED)
find_package(whale_interfaces REQUIRED)

# -------------------------------------------------------------------------
# Simulator orchestrator ROS node
# -------------------------------------------------------------------------

add_executable(
  sim_orchestrator_node
  src/sim_orchestrator_node.cpp
  src/sim_orchestrator_callbacks.cpp
  src/sim_orchestrator_interfaces.cpp
  src/sim_orchestrator_output.cpp
  src/sim_orchestrator_setup.cpp
)

# -------------------------------------------------------------------------
# Include paths for the existing simulator orchestrator code
# -------------------------------------------------------------------------

target_include_directories(
  sim_orchestrator_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /odyssey/components/manta_odyssey/include
    /odyssey/components/manta_odyssey/src/Common
    /odyssey/components/manta_odyssey/src/World
    /odyssey/components/manta_odyssey/src/Simulation
    /odyssey/components/manta_odyssey/src/Sensors
    /odyssey/components/manta_odyssey/src/Odyssey
)

# -------------------------------------------------------------------------
# ROS dependencies
# -------------------------------------------------------------------------

ament_target_dependencies(
  sim_orchestrator_node
  rclcpp
  std_msgs
  acoustics_interfaces
  manta_interfaces
  whale_interfaces
)

# -------------------------------------------------------------------------
# Existing Odyssey libraries
# -------------------------------------------------------------------------

target_link_libraries(
  sim_orchestrator_node
  "-Wl,--start-group"
  /odyssey/components/manta_odyssey/lib/libsimulation.a
  /odyssey/components/manta_odyssey/lib/libworld_sim.a
  /odyssey/components/manta_odyssey/lib/libsim_common.a
  /odyssey/components/manta_odyssey/lib/libsim_sensors.a
  /odyssey/components/manta_odyssey/lib/libodyssey.a
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
