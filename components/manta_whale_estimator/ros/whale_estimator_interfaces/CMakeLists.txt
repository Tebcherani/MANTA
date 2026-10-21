cmake_minimum_required(VERSION 3.8)
project(whale_estimator_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WhaleKalmanEstimate.msg"
)

ament_package()
