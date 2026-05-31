cmake_minimum_required(VERSION 3.22.1)
project(bumperbot_cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(bumperbot_msgs REQUIRED)

include_directories(include)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)

add_executable(simple_lifecycle_node src/simple_lifecycle_node.cpp)
ament_target_dependencies(
  simple_lifecycle_node
  rclcpp
  std_msgs
  rclcpp_lifecycle
)

add_executable(simple_parameter src/simple_parameter.cpp)
ament_target_dependencies(
  simple_parameter
  rclcpp
  std_msgs
  rclcpp_lifecycle
)

add_executable(turtlesim_kinematics src/turtlesim_kinematics.cpp)
ament_target_dependencies(
  turtlesim_kinematics
  rclcpp
  turtlesim
)

add_executable(simple_tf_kinematics src/simple_tf_kinematics.cpp)
ament_target_dependencies(
  simple_tf_kinematics
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  bumperbot_msgs
)

add_executable(simple_service_server src/simple_service_server.cpp)
ament_target_dependencies(
  simple_service_server
  rclcpp
  bumperbot_msgs
)

add_executable(simple_service_client src/simple_service_client.cpp)
ament_target_dependencies(
  simple_service_client
  rclcpp
  bumperbot_msgs
)

add_library(simple_action_server SHARED
  src/simple_action_server.cpp
)

target_include_directories(simple_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(simple_action_server PRIVATE
  SIMPLE_ACTION_SERVER_CPP_BUILDING_DLL
)

ament_target_dependencies(
  simple_action_server
  bumperbot_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
)

rclcpp_components_register_node(
  simple_action_server
  PLUGIN "bumperbot_cpp_examples::SimpleActionServer"
  EXECUTABLE simple_action_server_node
)

add_library(simple_action_client SHARED
  src/simple_action_client.cpp
)

target_include_directories(simple_action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(simple_action_client PRIVATE
  SIMPLE_ACTION_CLIENT_CPP_BUILDING_DLL
)

ament_target_dependencies(
  simple_action_client
  bumperbot_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
)

rclcpp_components_register_node(
  simple_action_client
  PLUGIN "bumperbot_cpp_examples::SimpleActionClient"
  EXECUTABLE simple_action_client_node
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    simple_publisher
    simple_subscriber
    simple_lifecycle_node
    simple_parameter
    turtlesim_kinematics
    simple_tf_kinematics
    simple_service_server
    simple_service_client
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    simple_action_server
    simple_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()