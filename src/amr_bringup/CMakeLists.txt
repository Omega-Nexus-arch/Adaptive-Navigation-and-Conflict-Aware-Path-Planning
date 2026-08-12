cmake_minimum_required(VERSION 3.8)
project(amr_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# ament index marker, so `ros2 pkg` and get_package_share_directory
# resolve this package from the install space.
install(FILES resource/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/send_goals.py
    scripts/demo_conflict.py
    scripts/demo_safety_override.py
    scripts/demo_slope_planning.py
    scripts/check_model_consistency.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(fleet_config_test test/test_fleet_loader.py)
  # Structural guard: no per-robot node may escape its namespace.
  ament_add_pytest_test(launch_namespacing_test test/test_launch_namespacing.py)
  # Guards the map -> base_footprint chain: the robot's start pose must appear
  # in exactly one link. See DESIGN_NOTES 8b.
  ament_add_pytest_test(frame_chain_test test/test_frame_chain.py)
  # Guards against a Node subclass shadowing a read-only rclpy property, which
  # raises only when the constructor runs. See DESIGN_NOTES 8c.
  ament_add_pytest_test(node_shadowing_test test/test_node_attribute_shadowing.py)
endif()

ament_package()
