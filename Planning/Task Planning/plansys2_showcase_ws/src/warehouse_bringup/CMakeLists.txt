cmake_minimum_required(VERSION 3.8)
project(warehouse_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
  rclcpp plansys2_domain_expert plansys2_executor plansys2_msgs
  plansys2_planner plansys2_problem_expert nav_msgs geometry_msgs
  visualization_msgs)

add_executable(mission_controller src/mission_controller.cpp)
ament_target_dependencies(mission_controller ${dependencies})
target_link_libraries(mission_controller ${YAML_CPP_LIBRARIES})

add_executable(fleet_monitor src/fleet_monitor.cpp)
ament_target_dependencies(fleet_monitor ${dependencies})
target_link_libraries(fleet_monitor ${YAML_CPP_LIBRARIES})

install(TARGETS mission_controller fleet_monitor
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
