cmake_minimum_required(VERSION 3.8)
project(tb3_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)

set(dependencies
  rclcpp
  plansys2_domain_expert
  plansys2_problem_expert
  plansys2_planner
  plansys2_executor
  plansys2_msgs
)

add_executable(planner_client src/planner_client.cpp)
ament_target_dependencies(planner_client ${dependencies})

install(TARGETS planner_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
