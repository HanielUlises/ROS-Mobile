cmake_minimum_required(VERSION 3.8)
project(nav_example)

# Default to C++17 (recommended for modern ROS 2)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ------------------------------
# Build the executable
# ------------------------------
add_executable(simple_nav_node
  src/simple_nav_node.cpp
)

ament_target_dependencies(simple_nav_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)

# Optional: if you later add more source files or want stricter warnings
# target_compile_options(simple_nav_node PRIVATE -Werror)

# ------------------------------
# Install targets
# ------------------------------
install(TARGETS
  simple_nav_node
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------
# Install directories (launch, urdf, config, rviz, etc.)
# ------------------------------
install(DIRECTORY
  launch
  urdf
  rviz
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Optional: install include directory if you have public headers
install(DIRECTORY
  include/
  DESTINATION include/
)

# ------------------------------
# Testing (optional but recommended)
# ------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # skip copyright check (enable when you add license)
  set(ament_cmake_copyright_FOUND TRUE)
  # skip cpplint if not in git repo or until license is added
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate and install package configuration
ament_package()