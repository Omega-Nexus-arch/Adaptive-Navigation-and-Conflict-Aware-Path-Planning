cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(LAYER_DEPENDENCIES
  amr_msgs rclcpp nav2_costmap_2d nav_msgs geometry_msgs pluginlib tf2 tf2_ros)

# The cost model is separated from the layer so it can be unit-tested against
# the generated warehouse elevation map without instantiating a costmap.
add_library(slope_cost_model SHARED src/slope_cost_model.cpp)
target_include_directories(slope_cost_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(amr_navigation_layers SHARED
  src/slope_layer.cpp
  src/fleet_trajectory_layer.cpp
)
target_include_directories(amr_navigation_layers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(amr_navigation_layers slope_cost_model)
ament_target_dependencies(amr_navigation_layers ${LAYER_DEPENDENCIES})

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(FILES costmap_plugins.xml DESTINATION share/${PROJECT_NAME})
install(
  TARGETS slope_cost_model amr_navigation_layers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(slope_cost_model amr_navigation_layers)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${LAYER_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Tested against the real generated map, not a synthetic fixture, so a world
  # change that broke the ramp costing fails the build.
  find_package(amr_gazebo REQUIRED)
  ament_add_gtest(test_slope_cost test/test_slope_cost_model.cpp)
  target_link_libraries(test_slope_cost slope_cost_model)
  target_compile_definitions(test_slope_cost PRIVATE
    AMR_NAVIGATION_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../amr_gazebo/maps"
  )
endif()

ament_package()
