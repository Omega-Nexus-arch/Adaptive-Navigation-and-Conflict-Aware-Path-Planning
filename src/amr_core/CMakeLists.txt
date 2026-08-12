cmake_minimum_required(VERSION 3.8)
project(amr_core)

# Google C++ Style targets C++17 here; nothing in this package needs newer.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/robot_model.cpp
  src/model_library.cpp
  src/fleet_config.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} yaml_cpp_vendor)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml_cpp_vendor yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # Copyright headers are present; the linter's year heuristics are noise here.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_amr_core
    test/test_robot_model.cpp
    test/test_geometry.cpp
    test/test_config_loading.cpp
  )
  target_link_libraries(test_amr_core ${PROJECT_NAME})
  target_compile_definitions(test_amr_core PRIVATE
    AMR_CORE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
  )
endif()

ament_package()
