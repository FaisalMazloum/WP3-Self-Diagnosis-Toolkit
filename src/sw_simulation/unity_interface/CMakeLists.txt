cmake_minimum_required(VERSION 3.8)
project(my_ros2unity)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(wheel_controller src/wheel_controller.cpp)
ament_target_dependencies(wheel_controller rclcpp geometry_msgs)

add_executable(wheel_pose src/wheel_pose.cpp)
ament_target_dependencies(wheel_pose rclcpp nav_msgs geometry_msgs)


install(TARGETS
  wheel_controller  
  wheel_pose
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
