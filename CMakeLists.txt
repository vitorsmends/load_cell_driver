cmake_minimum_required(VERSION 3.8)
project(load_cell_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(force_sensor_node
  src/force_sensor.cpp
)
ament_target_dependencies(force_sensor_node
  rclcpp
  std_msgs
)

add_executable(force_sensor src/main.cpp)
target_link_libraries(force_sensor force_sensor_node)
ament_target_dependencies(force_sensor
  rclcpp
  std_msgs
)

install(TARGETS
  force_sensor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  include
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_force_sensor test/test_force_sensor.cpp)
  if(TARGET test_force_sensor)
    target_link_libraries(test_force_sensor force_sensor_node)
    ament_target_dependencies(test_force_sensor
      rclcpp
      std_msgs
    )
  endif()
endif()

ament_package()
