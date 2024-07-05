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


include_directories(
  include
)

add_executable(server_cmd src/server_cmd.cpp)
ament_target_dependencies(
    server_cmd
    geometry_msgs
    rclcpp
    limo_msgs
    example_interfaces
)

add_executable(server_client src/server_client.cpp)
ament_target_dependencies(
    server_client
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
add_executable(topic_cmd src/topic_cmd.cpp)
ament_target_dependencies(
    topic_cmd
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
install(TARGETS
  server_cmd
  server_client
  limo_action_server
  limo_action_client
  topic_cmd
  listen_tf
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
