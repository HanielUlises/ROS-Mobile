cmake_minimum_required(VERSION 3.8)
project(tb3_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  plansys2_executor
  plansys2_msgs
)

add_executable(move_action    src/move_action.cpp)
add_executable(inspect_action src/inspect_action.cpp)
add_executable(report_action  src/report_action.cpp)

foreach(target move_action inspect_action report_action)
  ament_target_dependencies(${target} ${dependencies})
endforeach()

install(TARGETS move_action inspect_action report_action
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
