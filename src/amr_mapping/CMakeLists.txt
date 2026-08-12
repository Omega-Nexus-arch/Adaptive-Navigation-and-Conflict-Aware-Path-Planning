cmake_minimum_required(VERSION 3.8)
project(amr_mapping)

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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Mapping algorithms, free of rclcpp so they are unit-testable directly.
add_library(mapping_core SHARED
  src/selective_mapping.cpp
  src/map_fusion.cpp
)
target_include_directories(mapping_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(NODE_DEPENDENCIES amr_core amr_msgs rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

foreach(node selective_mapping map_fusion)
  add_executable(${node}_node src/${node}_node.cpp)
  target_link_libraries(${node}_node mapping_core)
  ament_target_dependencies(${node}_node ${NODE_DEPENDENCIES})
  install(TARGETS ${node}_node DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS mapping_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(mapping_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${NODE_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_amr_mapping test/test_selective_mapping.cpp)
  target_link_libraries(test_amr_mapping mapping_core)
endif()

ament_package()
