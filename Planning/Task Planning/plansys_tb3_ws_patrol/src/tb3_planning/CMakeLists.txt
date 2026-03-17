cmake_minimum_required(VERSION 3.8)
project(tb3_planning)

find_package(ament_cmake REQUIRED)

install(DIRECTORY pddl config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
