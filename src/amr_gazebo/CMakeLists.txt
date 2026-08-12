cmake_minimum_required(VERSION 3.8)
project(amr_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# ament index marker, so `ros2 pkg` and get_package_share_directory
# resolve this package from the install space.
install(FILES resource/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)

install(
  DIRECTORY worlds config maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/generate_world.py
    scripts/dynamic_obstacle_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(world_generation_test test/test_world_generation.py)
endif()

ament_package()
