#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <tf2_eigen/tf2_eigen.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true) //IS this what fixed the entire thing?
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "arm_move_group");

// // Set a target Pose
// auto const target_pose = []{
//   geometry_msgs::msg::Pose msg;
//   msg.orientation.x = -.495;
//   msg.orientation.y = -.505;
//   msg.orientation.z = .495;
//   msg.orientation.w = .505;

//   msg.position.x = 0.293;
//   msg.position.y = .022;
//   msg.position.z = 0.252;
//   return msg;
// }();


// Set a target Pose
auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
  msg.orientation.x = -.098;
  msg.orientation.y = -.374;
  msg.orientation.z = -.596;
  msg.orientation.w = .704;

  msg.position.x = 0.174;
  msg.position.y = -0.144;
  msg.position.z = 0.2;
  return msg;
}();

// Set a target Pose
// auto const target_pose = []{
//   geometry_msgs::msg::Pose msg;
//   msg.orientation.x = -.037;
//   msg.orientation.y = .96;
//   msg.orientation.z = -.274;
//   msg.orientation.w = -.036;

//   msg.position.x = 0.036;
//   msg.position.y = .135;
//   msg.position.z = 0.261;
//   return msg;
// }();



move_group_interface.setPoseTarget(target_pose);

// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}




  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
