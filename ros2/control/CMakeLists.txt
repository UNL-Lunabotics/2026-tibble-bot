cmake_minimum_required(VERSION 3.8)
project(control)

# Compiler settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies List
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_interface
  realtime_tools
  geometry_msgs
  std_msgs
  std_srvs
  nav_msgs
  tf2
  tf2_ros
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Phoenix 6 API (For Kraken X60/Talon FX)
find_package(ctre-phoenix6 REQUIRED)

# Create a single plugin library for both HWC and Controller
add_library(tibble_plugins SHARED
  src/tibble_hwc.cpp
  src/tibble_controller.cpp
)

target_include_directories(tibble_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link everything
# ament_target_dependencies handles the ROS packages
ament_target_dependencies(tibble_plugins PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Manually link the non-Ament Phoenix library
target_link_libraries(tibble_plugins PUBLIC
  ctre::phoenix6
)

# Export Plugins
pluginlib_export_plugin_description_file(hardware_interface config/tibble_hwc.xml)
pluginlib_export_plugin_description_file(controller_interface config/tibble_controller.xml)

# Install Logic
install(DIRECTORY include/ DESTINATION include)

install(TARGETS tibble_plugins
  EXPORT export_tibble_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Ament Exports
ament_export_targets(export_tibble_plugins HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} ctre-phoenix6)
ament_package()