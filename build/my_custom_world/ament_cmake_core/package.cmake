set(_AMENT_PACKAGE_NAME "my_custom_world")
set(my_custom_world_VERSION "0.0.0")
set(my_custom_world_MAINTAINER "mariem <mariem@example.com>")
set(my_custom_world_BUILD_DEPENDS "rclcpp" "nav_msgs" "tf2_ros" "tf2_geometry_msgs" "geometry_msgs")
set(my_custom_world_BUILDTOOL_DEPENDS "ament_cmake" "ament_cmake_python")
set(my_custom_world_BUILD_EXPORT_DEPENDS "rclcpp" "nav_msgs" "tf2_ros" "tf2_geometry_msgs" "geometry_msgs")
set(my_custom_world_BUILDTOOL_EXPORT_DEPENDS )
set(my_custom_world_EXEC_DEPENDS "rclpy" "numpy" "gazebo_ros" "turtlebot3_gazebo" "slam_toolbox" "nav2_map_server" "rclcpp" "nav_msgs" "tf2_ros" "tf2_geometry_msgs" "geometry_msgs")
set(my_custom_world_TEST_DEPENDS "ament_lint_auto" "ament_lint_common")
set(my_custom_world_GROUP_DEPENDS )
set(my_custom_world_MEMBER_OF_GROUPS )
set(my_custom_world_DEPRECATED "")
set(my_custom_world_EXPORT_TAGS)
list(APPEND my_custom_world_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
list(APPEND my_custom_world_EXPORT_TAGS "<ros2run><executable_script>lib/my_custom_world/map_merger</executable_script></ros2run>")
