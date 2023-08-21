cmake_minimum_required(VERSION 3.8)
project(demo_orchestration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ur_moveit_demo_msg REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

add_executable(demo_orchestration src/demo_orchestration.cpp)
target_include_directories(demo_orchestration PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  demo_orchestration
  ur_moveit_demo_msg
  "rclcpp"
  "tf2_msgs"
  "tf2_ros"
  )

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

install(TARGETS demo_orchestration
  DESTINATION lib/${PROJECT_NAME})

ament_package()
