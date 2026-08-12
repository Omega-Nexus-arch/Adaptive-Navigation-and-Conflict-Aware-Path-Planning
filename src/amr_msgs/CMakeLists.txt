cmake_minimum_required(VERSION 3.8)
project(amr_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryPoint.msg"
  "msg/PredictedTrajectory.msg"
  "msg/FleetTrajectories.msg"
  "msg/TrafficDirective.msg"
  "msg/SafetyStatus.msg"
  "msg/SensorHealth.msg"
  "msg/MapUpdateStats.msg"
  "msg/PayloadState.msg"
  "srv/SetSafetyOverride.srv"
  "srv/SetPayload.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The interface files carry their rationale in comments rather than a licence
  # header; the workspace licence is at the repository root. Matches every
  # other package here.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
