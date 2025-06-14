cmake_minimum_required(VERSION 3.8)
project(rm_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(fmt REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rm_ros_interfaces REQUIRED)

add_library(rm_robot_driver_plugin SHARED
  src/rm_driver_hwi.cpp
)
target_compile_features(rm_robot_driver_plugin PUBLIC cxx_std_17)
target_include_directories(rm_robot_driver_plugin PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rm_robot_driver_plugin
  pluginlib
  rcpputils
  hardware_interface
  controller_interface
  realtime_tools
  sensor_msgs
  rm_ros_interfaces
)

target_link_libraries(rm_robot_driver_plugin
  fmt::fmt
)

install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS rm_robot_driver_plugin
  EXPORT export_rm_robot_driver_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)

pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

if(BUILD_TESTING)
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
