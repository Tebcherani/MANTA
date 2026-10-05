cmake_minimum_required(VERSION 3.8)
project(whale_detection_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
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
# Whale-detection ROS node
# -------------------------------------------------------------------------

add_executable(
  whale_detection_node
  src/whale_detection_node.cpp
  src/whale_detection_callbacks.cpp
  src/whale_detection_interfaces.cpp
  src/whale_detection_setup.cpp
)

# -------------------------------------------------------------------------
# Include paths for the existing Odyssey simulator code
# -------------------------------------------------------------------------

target_include_directories(
  whale_detection_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(whale_detection_node PUBLIC cxx_std_17)

# -------------------------------------------------------------------------
# ROS dependencies
# -------------------------------------------------------------------------

ament_target_dependencies(
  whale_detection_node
  rclcpp
  manta_interfaces
  whale_interfaces
  std_msgs
)

# -------------------------------------------------------------------------
# Install executable into the ROS workspace
# -------------------------------------------------------------------------

install(
  TARGETS whale_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
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