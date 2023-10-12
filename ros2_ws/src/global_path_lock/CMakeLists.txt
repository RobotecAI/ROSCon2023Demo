cmake_minimum_required(VERSION 3.8)
project(global_path_lock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lock_service_msgs REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

add_executable(global_path_lock src/global_path_lock.cpp)

target_include_directories(global_path_lock PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  global_path_lock
  lock_service_msgs
  "rclcpp"
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS global_path_lock
  DESTINATION lib/${PROJECT_NAME})

ament_package()
