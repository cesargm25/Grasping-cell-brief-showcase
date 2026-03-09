cmake_minimum_required(VERSION 3.8)
project(ur_moveit_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

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

add_executable(joint_space_trajectory src/joint_space_trajectory.cpp)
ament_target_dependencies(joint_space_trajectory rclcpp moveit_ros_planning_interface)
install(TARGETS joint_space_trajectory DESTINATION lib/${PROJECT_NAME})

add_executable(end_pose src/end_pose.cpp)
ament_target_dependencies(end_pose rclcpp moveit_ros_planning_interface)
install(TARGETS end_pose DESTINATION lib/${PROJECT_NAME})

add_executable(cam_end_pose src/cam_end_pose.cpp)
ament_target_dependencies(cam_end_pose rclcpp geometry_msgs moveit_ros_planning_interface)
install(TARGETS cam_end_pose DESTINATION lib/${PROJECT_NAME})


ament_package()
