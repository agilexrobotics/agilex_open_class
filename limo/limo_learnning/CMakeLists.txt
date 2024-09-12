cmake_minimum_required(VERSION 3.8)
project(limo_learnning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(limo_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
include_directories(
  include
)

add_executable(limo_server src/limo_server.cpp)
ament_target_dependencies(
    limo_server
    geometry_msgs
    rclcpp
    limo_msgs
    example_interfaces
)

add_executable(limo_client src/limo_client.cpp)
ament_target_dependencies(
    limo_client
    geometry_msgs
    rclcpp
    limo_msgs
    example_interfaces
)


add_executable(limo_action_server src/limo_action_server.cpp)
ament_target_dependencies(
    limo_action_server
    geometry_msgs
    rclcpp
    rclcpp_action
    limo_msgs
    example_interfaces
)

add_executable(limo_action_client src/limo_action_client.cpp)
ament_target_dependencies(
    limo_action_client
    geometry_msgs
    rclcpp
    rclcpp_action
    limo_msgs
    example_interfaces
)
add_executable(limo_topic_cmd src/limo_topic_cmd.cpp)
ament_target_dependencies(
  limo_topic_cmd
    geometry_msgs
    nav_msgs
    rclcpp
    limo_msgs
    example_interfaces
)

add_executable(listen_tf src/listen_tf.cpp)
ament_target_dependencies(
    listen_tf
    geometry_msgs
    rclcpp
    example_interfaces
    tf2_ros
    tf2
)

add_executable(limo_scan src/limo_scan.cpp)
ament_target_dependencies(
  limo_scan
    geometry_msgs
    rclcpp
    example_interfaces
    sensor_msgs
)

add_executable(limo_scan_nav src/limo_scan_nav.cpp)
ament_target_dependencies(
  limo_scan_nav
    geometry_msgs
    rclcpp
    example_interfaces
    sensor_msgs
    rclcpp_action
    nav2_msgs
)

add_executable(multi_goal_nav src/multi_goal_nav.cpp)
ament_target_dependencies(
  multi_goal_nav
    geometry_msgs
    rclcpp
    example_interfaces
    sensor_msgs
    tf2_ros
    tf2
    nav2_msgs
    visualization_msgs
)

add_executable(explore src/explore.cpp)
ament_target_dependencies(
  explore
    geometry_msgs
    rclcpp
    example_interfaces
    sensor_msgs
    tf2_ros
    tf2
    nav2_msgs
    visualization_msgs
)

add_executable(record_path src/record_path.cpp)
ament_target_dependencies(
  record_path
    geometry_msgs
    rclcpp
    example_interfaces
    sensor_msgs
    tf2_ros
    tf2
    nav2_msgs
    tf2_geometry_msgs
)

add_executable(follow_path src/follow_path.cpp)
ament_target_dependencies(
  follow_path
    geometry_msgs
    rclcpp
    example_interfaces
    sensor_msgs
    tf2_ros
    tf2
    nav2_msgs
    tf2_geometry_msgs
)


install(TARGETS
  limo_server
  limo_client
  limo_action_server
  limo_action_client
  limo_topic_cmd
  listen_tf
  limo_scan
  limo_scan_nav
  multi_goal_nav
  explore
  record_path
  follow_path
  DESTINATION
 lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
