cmake_minimum_required(VERSION 3.16)
project(ptcam_robot LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  PiPCA9685
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  ptcam_robot
  SHARED
  src/hardware/ptcam_robot_system.cpp
)
target_compile_features(ptcam_robot PUBLIC cxx_std_17)
target_include_directories(ptcam_robot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/hardware/include>
$<INSTALL_INTERFACE:include/ptcam_robot>
)
target_link_libraries(ptcam_robot PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  PiPCA9685::PiPCA9685
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ptcam_robot.xml)

# INSTALL
install(
  DIRECTORY src/hardware/include/
  DESTINATION include/ptcam_robot
)
install(
  DIRECTORY src/description/launch src/description/urdf src/description/meshes
  DESTINATION share/ptcam_robot
)
install(
  DIRECTORY src/bringup/launch src/bringup/config
  DESTINATION share/ptcam_robot
)
install(TARGETS ptcam_robot
  EXPORT export_ptcam_robot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_ptcam_robot HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
