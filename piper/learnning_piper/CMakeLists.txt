cmake_minimum_required(VERSION 3.8)
project(learnning_piper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_visual_tools REQUIRED)


add_executable(move_target src/move_target.cpp)
ament_target_dependencies(
  move_target
    geometry_msgs
    rclcpp
    moveit_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    moveit_servo
)

add_executable(move_multi_target src/move_multi_target.cpp)
ament_target_dependencies(
  move_multi_target
    geometry_msgs
    rclcpp
    moveit_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    moveit_servo
)

add_executable(moveit_cpp_tutorial src/moveit_cpp_tutorial.cpp)
ament_target_dependencies(
  moveit_cpp_tutorial
    geometry_msgs
    rclcpp
    moveit_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    moveit_servo
)

install(TARGETS
  move_target
  move_multi_target
  moveit_cpp_tutorial
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
