cmake_minimum_required(VERSION 3.8)
project(mrs_coordination)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(node_dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

add_executable(map_merger_node src/map_merger_node.cpp)
ament_target_dependencies(map_merger_node ${node_dependencies})

add_executable(comms_link_node src/comms_link_node.cpp)
ament_target_dependencies(comms_link_node ${node_dependencies})

add_executable(explorer_node src/explorer_node.cpp)
ament_target_dependencies(explorer_node ${node_dependencies})

install(TARGETS
  map_merger_node
  comms_link_node
  explorer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
