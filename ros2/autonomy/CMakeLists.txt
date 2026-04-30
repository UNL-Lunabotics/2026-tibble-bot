cmake_minimum_required(VERSION 3.8)
project(autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/boot_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Behavior Tree XML files
install(DIRECTORY
  behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(DIRECTORY src/ 
  DESTINATION share/
)

ament_package()
