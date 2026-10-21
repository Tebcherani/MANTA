cmake_minimum_required(VERSION 3.8)
project(whale_estimator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(manta_interfaces REQUIRED)
find_package(whale_estimator_interfaces REQUIRED)
find_package(odyssey_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(whale_estimator_node src/whale_estimator_node.cpp src/whale_estimator.c)
target_include_directories(whale_estimator_node PRIVATE include include/whale_estimator)
target_compile_features(whale_estimator_node PUBLIC cxx_std_17)
ament_target_dependencies(whale_estimator_node rclcpp manta_interfaces whale_estimator_interfaces odyssey_interfaces std_msgs)
target_link_libraries(whale_estimator_node m)

install(TARGETS whale_estimator_node DESTINATION lib/${PROJECT_NAME})
ament_package()
