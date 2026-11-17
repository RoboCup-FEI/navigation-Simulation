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
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

add_executable(dynamic_obstacle_predictor src/dynamic_obstacle_predictor.cpp)
ament_target_dependencies(dynamic_obstacle_predictor
  rclcpp geometry_msgs nav_msgs sensor_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS dynamic_obstacle_predictor DESTINATION lib/${PROJECT_NAME})

add_library(predicted_path_obstacle_layer SHARED src/predicted_path_obstacle_layer.cpp)
target_include_directories(predicted_path_obstacle_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(predicted_path_obstacle_layer
  rclcpp nav2_costmap_2d pluginlib visualization_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d predicted_path_obstacle_layer.xml)
install(TARGETS predicted_path_obstacle_layer
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES predicted_path_obstacle_layer.xml DESTINATION share/${PROJECT_NAME})
ament_export_libraries(predicted_path_obstacle_layer)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
