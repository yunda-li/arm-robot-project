cmake_minimum_required(VERSION 3.8)
project(soarm_scripts_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(moveit_test src/moveit_test.cpp)
ament_target_dependencies(moveit_test 
  rclcpp 
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
  geometry_msgs
  )

add_executable(pose_checker src/pose_checker.cpp)
ament_target_dependencies(pose_checker 
  rclcpp 
  moveit_ros_planning_interface 
  moveit_ros_planning
  tf2_ros
  tf2_eigen
  tf2
  )

  add_executable(position_checker src/position_checker.cpp)
ament_target_dependencies(position_checker 
  rclcpp 
  moveit_ros_planning_interface 
  moveit_ros_planning
  tf2_ros
  tf2_eigen
  )


install(TARGETS
  moveit_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pose_checker
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  position_checker
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
