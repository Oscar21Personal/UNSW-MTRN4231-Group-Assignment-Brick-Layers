cmake_minimum_required(VERSION 3.8)
project(motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interfaces REQUIRED)

add_executable(move_to_location src/move_to_location.cpp)
ament_target_dependencies(move_to_location rclcpp tf2 tf2_ros moveit_ros_planning_interface interfaces)

add_executable(path_planning src/path_planning.cpp)
ament_target_dependencies(path_planning rclcpp std_msgs interfaces)

install(TARGETS 
  move_to_location
  path_planning
DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
