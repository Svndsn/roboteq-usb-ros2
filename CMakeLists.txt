# Minimum CMake version required
cmake_minimum_required(VERSION 3.5)

# Project name
project(roboteq_node_ros2)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Generate message and service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelsMsg.msg"
  "srv/Actuators.srv"
  "srv/SendCANCommand.srv"
  DEPENDENCIES std_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# Add shared library
add_library(roboteq_node_lib SHARED
  src/rosRoboteqDrv/rosRoboteqDrv.cpp
  src/roboteqCom/roboteqCom.cpp
  src/roboteqCom/roboteqThread.cpp
  src/serialConnector/serialPort.cpp
)
ament_target_dependencies(roboteq_node_lib 
  rclcpp std_msgs 
  geometry_msgs 
  rosidl_typesupport_cpp
)

# Add executable
add_executable(roboteq_node_ros2_executable
  src/rosRoboteqDrv/main.cpp
  src/rosRoboteqDrv/rosRoboteqDrv.cpp
  src/roboteqCom/roboteqCom.cpp
  src/roboteqCom/roboteqThread.cpp
  src/serialConnector/serialPort.cpp
)
ament_target_dependencies(roboteq_node_ros2_executable
  rclcpp
  std_msgs
  geometry_msgs
  rosidl_typesupport_cpp
)
target_link_libraries(
  roboteq_node_ros2_executable
  roboteq_node_lib
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Install targets
install(TARGETS
  roboteq_node_ros2_executable
  roboteq_node_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Mark the package as an ament package
ament_package()