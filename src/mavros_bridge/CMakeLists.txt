cmake_minimum_required(VERSION 3.22.1)
project(mavros_bridge LANGUAGES C CXX)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)

include_directories(
  include
)

# Build the component library
add_library(mavros_bridge_component SHARED
  src/mavros_bridge_component.cpp
)

# Register the component
rclcpp_components_register_nodes(mavros_bridge_component 
  "uosm::mavros::MavrosBridgeComponent")

# Dependencies
ament_target_dependencies(mavros_bridge_component
  rclcpp
  rclcpp_components
  nav_msgs
  mavros_msgs
)

# Install
install(TARGETS
  mavros_bridge_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch and other files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()