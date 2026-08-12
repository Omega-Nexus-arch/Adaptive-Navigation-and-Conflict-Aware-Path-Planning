cmake_minimum_required(VERSION 3.8)
project(amr_sensor_bsp)

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
find_package(sensor_msgs REQUIRED)

# Validation logic, free of rclcpp so it is unit-testable without a node.
add_library(sensor_bsp SHARED src/sensor_validators.cpp)
target_include_directories(sensor_bsp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sensor_bsp amr_core)

add_executable(bsp_validation_node src/bsp_validation_node.cpp)
target_link_libraries(bsp_validation_node sensor_bsp)
ament_target_dependencies(bsp_validation_node amr_core amr_msgs rclcpp sensor_msgs)

install(TARGETS bsp_validation_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS sensor_bsp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(sensor_bsp)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(amr_core amr_msgs rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_sensor_bsp test/test_sensor_validators.cpp)
  target_link_libraries(test_sensor_bsp sensor_bsp)
  ament_target_dependencies(test_sensor_bsp amr_core)
endif()

ament_package()
