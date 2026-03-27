cmake_minimum_required(VERSION 3.8)
project(control)

# Compiler settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20) # THIS HAS TO BE 20 OR MORE FOR PHOENIX6
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
  phoenix6
  Threads
  SDL2
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

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
ament_target_dependencies(tibble_plugins PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Link directly to the compiled CTRE library in /usr/lib
target_link_libraries(tibble_plugins PUBLIC
  phoenix6
  pthread
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
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()