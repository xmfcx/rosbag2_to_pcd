cmake_minimum_required(VERSION 3.5)
project(rosbag2_to_pcd)

## Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  SYSTEM ${PCL_INCLUDE_DIRS}
)

## Targets
ament_auto_add_library(rosbag2_to_pcd_node_component SHARED
  src/include/rosbag2_to_pcd.hpp
  src/rosbag2_to_pcd.cpp
)

rclcpp_components_register_node(rosbag2_to_pcd_node_component
  PLUGIN "rosbag2_to_pcd::Rosbag2ToPcdNode"
  EXECUTABLE rosbag2_to_pcd_node
)

## Tests
if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Package
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
