cmake_minimum_required(VERSION 3.8)
project(spatial_maps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/network_navigation_node.py
  scripts/map_publisher_node.py
  scripts/poi_publisher_node.py
  scripts/poi_nav_node.py
  scripts/poi_click_node.py
  scripts/joint_state_relay.py
  scripts/odom_tf_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install map files
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install network files
install(DIRECTORY network
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install mesh files
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
