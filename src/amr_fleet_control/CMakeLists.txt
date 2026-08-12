cmake_minimum_required(VERSION 3.8)
project(amr_fleet_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_core REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(NODE_DEPENDENCIES
  amr_core
  amr_msgs
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# ---------------------------------------------------------------------------
# Algorithm library: deliberately free of rclcpp so it can be unit-tested
# without spinning a node.
# ---------------------------------------------------------------------------
add_library(fleet_control_core SHARED
  src/motion_smoother.cpp
  src/safety_monitor.cpp
  src/trajectory.cpp
  src/traffic_manager.cpp
)
target_include_directories(fleet_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fleet_control_core amr_core)

# ---------------------------------------------------------------------------
# Nodes: thin ROS adapters around the library above.
# ---------------------------------------------------------------------------
foreach(node velocity_smoother safety_override trajectory_broadcaster traffic_control)
  add_executable(${node}_node src/${node}_node.cpp)
  target_link_libraries(${node}_node fleet_control_core)
  ament_target_dependencies(${node}_node ${NODE_DEPENDENCIES})
  install(TARGETS ${node}_node DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS fleet_control_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(fleet_control_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${NODE_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_fleet_control
    test/test_motion_smoother.cpp
    test/test_safety_monitor.cpp
    test/test_traffic_manager.cpp
  )
  target_include_directories(test_fleet_control PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(test_fleet_control fleet_control_core)
  ament_target_dependencies(test_fleet_control amr_core)
endif()

ament_package()
