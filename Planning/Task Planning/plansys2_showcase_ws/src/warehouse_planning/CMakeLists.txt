cmake_minimum_required(VERSION 3.8)
project(warehouse_planning)

find_package(ament_cmake REQUIRED)

install(DIRECTORY pddl config maps DESTINATION share/${PROJECT_NAME})

ament_package()
