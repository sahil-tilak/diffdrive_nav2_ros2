cmake_minimum_required(VERSION 3.8)
project(diffdrive_nav2_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(urdf REQUIRED)
# find_package(xacro REQUIRED)
# find_package(robot_state_publisher REQUIRED)
# find_package(joint_state_publisher REQUIRED)

# --- perception (obstacle detector) ---
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

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

install(DIRECTORY
  urdf
  launch
  worlds
  maps
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/moving_obstacle.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- perception (obstacle detector) ---
# Clustering library: plain C++17, no ROS (can move to its own package later).
add_library(scan_clustering STATIC src/perception/scan_clustering.cpp)
target_include_directories(scan_clustering PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(scan_clustering PUBLIC cxx_std_17)

add_executable(obstacle_detector_node src/perception/obstacle_detector_node.cpp)
target_link_libraries(obstacle_detector_node scan_clustering)
ament_target_dependencies(obstacle_detector_node
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs message_filters)
install(TARGETS obstacle_detector_node DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/perception_record.py
  scripts/perception_analyze.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_clustering test/test_scan_clustering.cpp)
  target_link_libraries(test_scan_clustering scan_clustering)
endif()

ament_package()
