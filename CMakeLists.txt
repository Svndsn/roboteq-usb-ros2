cmake_minimum_required(VERSION 3.5)
project(roboteq_node_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Ensure message and service generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelsMsg.msg"
  "srv/Actuators.srv"
  "srv/SendCANCommand.srv"
  DEPENDENCIES std_msgs
)

# Ensure the generated interfaces are available
ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Ensure message dependencies are correctly set up
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}__rosidl_generator_cpp
)

# Library
add_library(roboteq_node_lib SHARED
  src/rosRoboteqDrv/rosRoboteqDrv.cpp
  src/roboteqCom/roboteqCom.cpp
  src/roboteqCom/roboteqThread.cpp
  src/serialConnector/serialPort.cpp
)

ament_target_dependencies(roboteq_node_lib 
  rclcpp std_msgs geometry_msgs rosidl_typesupport_cpp
)

# Make sure messages are built before compiling the library
add_dependencies(roboteq_node_lib ${PROJECT_NAME}__rosidl_generator_cpp)

target_link_libraries(roboteq_node_lib ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Main executable
add_executable(roboteq_node_ros2_executable
  src/rosRoboteqDrv/main.cpp
  src/rosRoboteqDrv/rosRoboteqDrv.cpp
  src/roboteqCom/roboteqCom.cpp
  src/roboteqCom/roboteqThread.cpp
  src/serialConnector/serialPort.cpp
)

ament_target_dependencies(roboteq_node_ros2_executable
  rclcpp std_msgs geometry_msgs rosidl_typesupport_cpp
)

add_dependencies(roboteq_node_ros2_executable ${PROJECT_NAME}__rosidl_generator_cpp)

target_link_libraries(roboteq_node_ros2_executable
  roboteq_node_lib
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Ensure the install path includes the generated message headers
install(DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/roboteq_node_ros2
  DESTINATION include/
)

# Install rules
install(TARGETS
  roboteq_node_ros2_executable
  roboteq_node_lib
  EXPORT roboteq_node_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  roboteq_node_ros2_executable
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Export dependencies
ament_export_include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp include)
ament_export_libraries(roboteq_node_lib)
ament_export_dependencies(rosidl_default_runtime rclcpp std_msgs geometry_msgs)

ament_package()