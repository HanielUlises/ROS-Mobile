cmake_minimum_required(VERSION 3.8)
project(robot_plansys)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  plansys2_executor
  plansys2_domain_expert
  plansys2_planner
  plansys2_problem_expert
  plansys2_msgs
)

include_directories(include)

# Move action
add_executable(move_action src/move_action.cpp)
ament_target_dependencies(move_action ${dependencies})

# Pick action
add_executable(pick_action src/pick_action.cpp)
ament_target_dependencies(pick_action ${dependencies})

# Place action
add_executable(place_action src/place_action.cpp)
ament_target_dependencies(place_action ${dependencies})

# Controller
add_executable(robot_controller src/robot_controller.cpp)
ament_target_dependencies(robot_controller ${dependencies})

install(TARGETS
  move_action
  pick_action
  place_action
  robot_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  pddl
  urdf
  config
  worlds
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
