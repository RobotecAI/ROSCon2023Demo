cmake_minimum_required(VERSION 3.8)
project(ur_palletization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(pilz_industrial_motion_planner REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(moveit_core)
add_library(gripper src/gripper.cpp src/gripper.h)
add_library(vision src/vision.cpp src/vision.h)
add_library(taskConstructor src/taskConstructor.cpp src/taskConstructor.h)
add_library(utils src/utils.cpp src/utils.h)

add_executable(palletizationNode src/palletizationNode.cpp)
target_link_libraries(palletizationNode gripper vision taskConstructor utils)
#target_include_directories(ur_moveit_demo PUBLIC
#$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#$<INSTALL_INTERFACE:include>)
target_include_directories(palletizationNode PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  palletizationNode
  "rclcpp"
  "moveit_ros_planning_interface"
  "pilz_industrial_motion_planner"
  "control_msgs"
  "tf2_msgs"
  "vision_msgs")

ament_target_dependencies(
  gripper
  "rclcpp"
  "rclcpp_action"
  "control_msgs"
  )

ament_target_dependencies(
  vision
  "rclcpp"
  "rclcpp_action"
  "control_msgs"
  "vision_msgs"
  "tf2_msgs"
  "tf2_geometry_msgs"
  "tf2_eigen"
  )

ament_target_dependencies(
  taskConstructor
  "rclcpp"
  "moveit_ros_planning_interface"
  "pilz_industrial_motion_planner"
  "control_msgs"
  "tf2_msgs"
  "vision_msgs")

ament_target_dependencies(
  utils
  "rclcpp"
  "moveit_ros_planning_interface"
  "pilz_industrial_motion_planner"
  "control_msgs"
  "tf2_msgs"
  "vision_msgs")

install(TARGETS palletizationNode
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

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
