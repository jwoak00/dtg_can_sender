cmake_minimum_required(VERSION 3.8)
project(dtg_can_sender)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(autoware_vehicle_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/dtg_can_sender.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "dtg_can_sender::DtgCanSender"
  EXECUTABLE ${PROJECT_NAME}_node
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  can_msgs
  autoware_control_msgs
  autoware_auto_vehicle_msgs
  autoware_adapi_v1_msgs
  autoware_vehicle_msgs
  tier4_vehicle_msgs
  nav_msgs
  std_msgs
  geometry_msgs
)