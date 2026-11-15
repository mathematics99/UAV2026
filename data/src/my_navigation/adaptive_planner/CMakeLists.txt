cmake_minimum_required(VERSION 3.8)
project(adaptive_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(adaptive_planner_node
  src/adaptive_planner.cpp
)

ament_target_dependencies(adaptive_planner_node
  rclcpp
  std_msgs
)

install(TARGETS
  adaptive_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
