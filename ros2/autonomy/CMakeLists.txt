cmake_minimum_required(VERSION 3.8)
project(autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(interfaces REQUIRED)

add_library(state_manager_autonomy_plugin SHARED src/state_manager_autonomy.cpp)
ament_target_dependencies(state_manager_autonomy_plugin rclcpp behaviortree_cpp interfaces)

add_executable(autonomy_executor src/autonomy_executor.cpp)
ament_target_dependencies(autonomy_executor rclcpp behaviortree_cpp ament_index_cpp)

install(TARGETS
  state_manager_autonomy_plugin
  autonomy_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
