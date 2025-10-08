cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # Add this if not already added

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StartEndPoints.msg"
  "msg/PartSize.msg"
  "srv/ArmPose.srv"
  "srv/ArduinoCall.srv"
  "srv/GetPartSize.srv"
  DEPENDENCIES std_msgs  # Ensure std_msgs dependency is included
)

# ament package macros
ament_export_dependencies(rosidl_default_runtime)
ament_package()
