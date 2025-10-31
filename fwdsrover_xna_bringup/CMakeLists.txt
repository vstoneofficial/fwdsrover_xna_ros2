cmake_minimum_required(VERSION 3.8)
project(fwdsrover_xna_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

add_executable(pub_odom src/pub_odom.cpp)
ament_target_dependencies(
  pub_odom
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  nav_msgs
  tf2_ros
  tf2
)

add_executable(rover_sim_node src/rover_sim_node.cpp)
ament_target_dependencies(
  rover_sim_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)

add_executable(gazebo_odom_bridge src/gazebo_odom_bridge.cpp)
ament_target_dependencies(
  gazebo_odom_bridge
  rclcpp
  nav_msgs
  geometry_msgs
  gazebo_msgs
  tf2_ros
)

install(TARGETS
  pub_odom
  rover_sim_node
  gazebo_odom_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  rviz
  config
  params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

