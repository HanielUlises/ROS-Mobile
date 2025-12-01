cmake_minimum_required(VERSION 3.8)
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
find_package(bumperbot_msgs REQUIRED)

include_directories(include)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add executables
add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)

add_executable(simple_lifecycle_node src/simple_lifecycle_node.cpp)
ament_target_dependencies(simple_lifecycle_node rclcpp std_msgs rclcpp_lifecycle)

add_executable(simple_parameter src/simple_parameter.cpp)
ament_target_dependencies(simple_parameter rclcpp std_msgs rclcpp_lifecycle)

add_executable(turtlesim_kinematics src/turtlesim_kinematics.cpp)
ament_target_dependencies(turtlesim_kinematics rclcpp turtlesim)

add_executable(simple_tf_kinematics src/simple_tf_kinematics.cpp)
ament_target_dependencies(simple_tf_kinematics rclcpp tf2 tf2_ros geometry_msgs bumperbot_msgs)

add_executable(simple_service_server src/simple_service_server.cpp)
ament_target_dependencies(simple_service_server rclcpp bumperbot_msgs)

add_executable(simple_service_client src/simple_service_client.cpp)
ament_target_dependencies(simple_service_client rclcpp bumperbot_msgs)


install(
  DIRECTORY include
  DESTINATION include
)

# Install executables
install(TARGETS
  simple_publisher
  simple_subscriber
  simple_lifecycle_node
  simple_tf_kinematics
  simple_parameter
  turtlesim_kinematics
  simple_service_server
  simple_service_client 
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()