cmake_minimum_required(VERSION 3.8)
project(teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rcl_interfaces REQUIRED)   # <-- ADD
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# NOTE: If you keep the simplified driver interface I gave you,
# you no longer need sensor_msgs or control_msgs here.
# find_package(sensor_msgs REQUIRED)
# find_package(control_msgs REQUIRED)

add_executable(qbot_platform_driver_interface src/qbot_platform_driver_interface.cpp)
target_link_libraries(qbot_platform_driver_interface -lquanser_communications)
ament_target_dependencies(qbot_platform_driver_interface
  rclcpp
  rcl_interfaces   # <-- ADD
  std_msgs
  geometry_msgs
)

add_executable(command src/command.cpp)
target_link_libraries(command -lquanser_devices -lquanser_communications -lquanser_runtime -lquanser_common)
ament_target_dependencies(command
  rclcpp
  geometry_msgs
  std_msgs         # <-- ADD (Bool + ColorRGBA)
)

install(TARGETS
  qbot_platform_driver_interface
  command
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()