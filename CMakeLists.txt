cmake_minimum_required(VERSION 3.5)
project(roboteq_node_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Add the message and service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelsMsg.msg"
  "srv/Actuators.srv"
  "srv/SendCANCommand.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

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


install(TARGETS
  roboteq_node_ros2_executable
  roboteq_node_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()