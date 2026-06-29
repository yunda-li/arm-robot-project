cmake_minimum_required(VERSION 3.8)
project(soarm_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

find_package(rcl_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(joint_tester src/joint_tester.cpp)
ament_target_dependencies(joint_tester rclcpp std_msgs trajectory_msgs urdf kdl_parser)

add_executable(joint_sweeper src/joint_sweeper.cpp)
ament_target_dependencies(joint_sweeper rclcpp rclcpp_lifecycle std_msgs trajectory_msgs urdf)

# Include directories (Crucial for KDL headers to resolve)
target_include_directories(joint_tester PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${orocos_kdl_INCLUDE_DIRS}
)

target_link_libraries(joint_tester
  kdl_parser::kdl_parser
  ${orocos_kdl_LIBRARIES}
)

install(TARGETS
  joint_tester joint_sweeper
  DESTINATION lib/${PROJECT_NAME}
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
