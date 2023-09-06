cmake_minimum_required(VERSION 3.8)
project(blind_path_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ur_moveit_demo_msg REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (tf2_eigen REQUIRED)
find_package(blind_path_follower_msgs)

add_executable(blind_path_follower src/follow_path.cpp)

ament_target_dependencies( blind_path_follower
    "blind_path_follower_msgs"
    "rclcpp"
    "tf2_msgs"
    "tf2_ros"
    "geometry_msgs"
    "nav2_msgs"
    "tf2_eigen"
)
target_link_libraries (blind_path_follower Eigen3::Eigen)


install(TARGETS blind_path_follower
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()