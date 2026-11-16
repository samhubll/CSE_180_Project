cmake_minimum_required(VERSION 3.8)
project(search)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(navigation REQUIRED)

add_library(perception src/perception.cpp)
target_include_directories(perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(perception geometry_msgs sensor_msgs vision_msgs)

add_executable(search src/search.cpp)
target_link_libraries(search perception)
ament_target_dependencies(search
  rclcpp rclcpp_action geometry_msgs sensor_msgs vision_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs navigation)
install(TARGETS search DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nlohmann_json REQUIRED)
  find_package(nav2_msgs REQUIRED)
  ament_add_gtest(test_perception test/test_perception.cpp)
  target_link_libraries(test_perception perception)
  ament_target_dependencies(test_perception tf2 tf2_ros tf2_geometry_msgs)
  add_executable(search_live_probe test/live_probe.cpp)
  ament_target_dependencies(search_live_probe
    rclcpp sensor_msgs vision_msgs visualization_msgs geometry_msgs
    tf2 tf2_ros tf2_geometry_msgs)
  install(TARGETS search_live_probe DESTINATION lib/${PROJECT_NAME})
  add_executable(validation_motion test/validation_motion.cpp)
  ament_target_dependencies(validation_motion rclcpp rclcpp_action geometry_msgs navigation)
  install(TARGETS validation_motion DESTINATION lib/${PROJECT_NAME})
  add_executable(range_probe test/range_probe.cpp)
  target_link_libraries(range_probe perception nlohmann_json::nlohmann_json)
  ament_target_dependencies(range_probe
    rclcpp geometry_msgs sensor_msgs vision_msgs tf2 tf2_ros tf2_geometry_msgs)
  install(TARGETS range_probe DESTINATION lib/${PROJECT_NAME})
  ament_add_gtest(test_coverage_plan test/test_coverage_plan.cpp)
  target_include_directories(test_coverage_plan PRIVATE include)
  add_executable(coverage_map test/coverage_map.cpp)
  target_include_directories(coverage_map PRIVATE include)
  install(TARGETS coverage_map DESTINATION lib/${PROJECT_NAME})
  add_executable(check_coverage_paths test/check_coverage_paths.cpp)
  target_include_directories(check_coverage_paths PRIVATE include)
  target_link_libraries(check_coverage_paths nlohmann_json::nlohmann_json)
  ament_target_dependencies(check_coverage_paths rclcpp rclcpp_action nav2_msgs geometry_msgs)
  install(TARGETS check_coverage_paths DESTINATION lib/${PROJECT_NAME})
  add_executable(validation_initial_pose test/validation_initial_pose.cpp)
  ament_target_dependencies(validation_initial_pose rclcpp geometry_msgs)
  install(TARGETS validation_initial_pose DESTINATION lib/${PROJECT_NAME})
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
