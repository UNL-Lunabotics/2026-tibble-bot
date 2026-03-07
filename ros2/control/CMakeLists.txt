cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# LibSerial nonsense
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

add_library(
  tibble_hwc
  SHARED
  src/tibble_hwc.cpp
)

target_compile_features(tibble_hwc PUBLIC cxx_std_17)

# IMPORTANT: include/ is now directly under this package root
target_include_directories(tibble_hwc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# LibSerial headers (PRIVATE: only your .cpp needs them)
target_include_directories(tibble_hwc PRIVATE
  ${LIBSERIAL_INCLUDE_DIRS}
)

# LibSerial link dirs + libs
target_link_directories(tibble_hwc PRIVATE
  ${LIBSERIAL_LIBRARY_DIRS}
)

target_link_libraries(tibble_hwc PRIVATE
  ${LIBSERIAL_LIBRARIES}
)

ament_target_dependencies(
  tibble_hwc PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface config/tibble_hwc.xml)

# -----------------------------
# Controller plugin (Jazzy)
# -----------------------------
add_library(
  tibble_controller
  SHARED
  src/tibble_controller.cpp
)

target_compile_features(tibble_controller PUBLIC cxx_std_17)

target_include_directories(tibble_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  tibble_controller PUBLIC
  rclcpp
  rclcpp_lifecycle
  controller_interface
  pluginlib
  realtime_tools
  geometry_msgs
  std_msgs
  std_srvs
  nav_msgs
  tf2
  tf2_ros
)

pluginlib_export_plugin_description_file(controller_interface config/tibble_controller.xml)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the shared library
install(TARGETS tibble_hwc
  EXPORT export_tibble_hwc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install controller shared library
install(TARGETS tibble_controller
  EXPORT export_tibble_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin XML so hardware_interface can find it
install(
  # FILES config/tibble_hwc.xml
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_tibble_hwc HAS_LIBRARY_TARGET)
ament_export_targets(export_tibble_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
