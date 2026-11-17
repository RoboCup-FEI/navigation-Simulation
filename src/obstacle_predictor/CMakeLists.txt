cmake_minimum_required(VERSION 3.8)
project(obstacle_predictor)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(dynamic_obstacle_predictor src/dynamic_obstacle_predictor.cpp)
ament_target_dependencies(dynamic_obstacle_predictor
  rclcpp geometry_msgs nav_msgs sensor_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS dynamic_obstacle_predictor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
