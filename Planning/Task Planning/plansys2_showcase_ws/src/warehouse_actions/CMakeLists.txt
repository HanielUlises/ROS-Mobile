cmake_minimum_required(VERSION 3.8)
project(warehouse_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
  rclcpp rclcpp_lifecycle plansys2_executor plansys2_msgs
  geometry_msgs nav_msgs sensor_msgs visualization_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(move_action src/move_action.cpp)
ament_target_dependencies(move_action ${dependencies})
target_link_libraries(move_action ${YAML_CPP_LIBRARIES})

add_executable(manipulate_action src/manipulate_action.cpp)
ament_target_dependencies(manipulate_action ${dependencies})
target_link_libraries(manipulate_action ${YAML_CPP_LIBRARIES})

install(TARGETS move_action manipulate_action
  DESTINATION lib/${PROJECT_NAME})

ament_package()
