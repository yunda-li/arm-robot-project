#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection/collision_common.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define RAD_CONV .017453

using moveit::planning_interface::MoveGroupInterface;
using geometry_msgs::msg::Pose;

bool planAndExecutePose(MoveGroupInterface &move_group_interface, const rclcpp::Logger &logger, const Pose &target_pose){

  tf2::Quaternion q(
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z,
    target_pose.orientation.w);
  q.normalize();

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  RCLCPP_INFO(logger, "Initial Goal orientation (RPY, degrees): roll=%f pitch=%f yaw=%f",
    roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

  //Orientation relaxing, test with setTargetPose() and setTargetPosition()
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = "gripper";
  ocm.header.frame_id = move_group_interface.getPlanningFrame();
  ocm.orientation = target_pose.orientation;

  //Orientation deviation from given 
  ocm.absolute_x_axis_tolerance = 90*RAD_CONV; //I think Roll might be the most important for keeping gripper aligned, but with correction can be loose
  ocm.absolute_y_axis_tolerance = 60*RAD_CONV;
  ocm.absolute_z_axis_tolerance = 60*RAD_CONV;
  ocm.weight = 1.0;
  
  moveit_msgs::msg::Constraints constraints;
  constraints.orientation_constraints.push_back(ocm);
  move_group_interface.setPathConstraints(constraints);

  // move_group_interface.setPoseTarget(target_pose, "gripper");
  move_group_interface.setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z, "gripper");

  MoveGroupInterface::Plan plan;
  bool ok = static_cast<bool>(move_group_interface.plan(plan));

  if (ok){
    move_group_interface.execute(plan);
    return true;
  }
  else{
    RCLCPP_WARN(logger, "Restrained Position Target Failed");
    return false;
  }
}

void planAndExecuteJoints(MoveGroupInterface &move_group_interface, const rclcpp::Logger &logger){
  auto const [success, plan] = [&move_group_interface](){
    MoveGroupInterface::Plan msg;
    auto ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  //Maybe add FK? Probably not.
  if (success){
    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Joint Target Reached");
  }
  else{
    RCLCPP_WARN(logger, "Joint Target Failed");
  }

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("hello_moveit");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm_move_group");

  // move_group_interface.setGoalPositionTolerance(0.05); 

  std::thread spin_thread([&node]() { rclcpp::spin(node); });

  moveit::core::RobotStatePtr real_current_state;
  std::vector<double> joint_values;
  bool got_real_state = false;

  for (int attempt = 0; attempt < 50; ++attempt) {  // up to ~5 seconds
    real_current_state = move_group_interface.getCurrentState();
    const moveit::core::JointModelGroup* jmg_check =
        real_current_state->getJointModelGroup("arm_move_group");
    real_current_state->copyJointGroupPositions(jmg_check, joint_values);

    // Heuristic: real state is very unlikely to be EXACTLY all zero
    bool all_zero = true;
    for (double v : joint_values) {
      if (std::abs(v) > 1e-6) { all_zero = false; break; }
    }

    if (!all_zero) {
      got_real_state = true;
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  if (!got_real_state) {
    RCLCPP_WARN(logger, "Current state still reads all-zero after waiting — may be genuine, or monitor issue persists.");
  }

  for (size_t i = 0; i < joint_values.size(); ++i) {
    RCLCPP_INFO(logger, "Joint %zu: %f", i, joint_values[i]);
  }

  // // HOME
  // auto const target_pose = []{
  //   geometry_msgs::msg::Pose msg;
  //   msg.position.x = 0.104;
  //   msg.position.y = 0.022;
  //   msg.position.z = 0.217;

  //   msg.orientation.x = -.024;
  //   msg.orientation.y = .994;
  //   msg.orientation.z = .003;
  //   msg.orientation.w = -.110;

  //   return msg;
  // }();

  //PRE-PICK
  // auto target_pose = []{
  //   geometry_msgs::msg::Pose msg;
  //   msg.position.x = 0.0814;
  //   msg.position.y = 0.2331;
  //   msg.position.z = 0.2168;

  //   msg.orientation.x = .3863;
  //   msg.orientation.y = -.0007;
  //   msg.orientation.z = -.2080;
  //   msg.orientation.w = .8986;

  //   return msg;
  // }();

  // PRE-PLACE
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.206;
    msg.position.y = -0.206;
    msg.position.z = 0.136;

    msg.orientation.x = -.691;
    msg.orientation.y = -.189;
    msg.orientation.z = .211;
    msg.orientation.w = .665;

    return msg;
  }();  

  // auto target_pose = []{
  //   geometry_msgs::msg::Pose msg;
  //   msg.position.x = 0.050981;
  //   msg.position.y = 0.230670;
  //   msg.position.z = 0.154420;

  //   msg.orientation.x = .13687;
  //   msg.orientation.y = -.016418;
  //   msg.orientation.z = .00065949;
  //   msg.orientation.w = .99045;

  //   return msg;
  // }();  

  // std::vector<double> joints = {-61*RAD_CONV, 37*RAD_CONV, -51*RAD_CONV, 20*RAD_CONV, 101*RAD_CONV};
  // move_group_interface.setJointValueTarget(joints);
  // planAndExecuteJoints(move_group_interface, logger);

  planAndExecutePose(move_group_interface, logger, target_pose);

  //Relative motion, test orientation adjustment
  rclcpp::sleep_for(std::chrono::milliseconds(200)); // let CurrentStateMonitor catch up post-execute
  auto current_pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(logger, "Current pose for Cartesian start: x=%f y=%f z=%f",
      current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

  //Correct wrist in Joint Space here, Horizontal flat is 93 or -87, Vertical is 0
  std::vector<double> joint_group_positions;
  const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("arm_move_group");
  move_group_interface.getCurrentState()->copyJointGroupPositions(joint_model_group, joint_group_positions);

  int wrist_roll_joint_index = 4;
  //Horizontal flat is 93 or -87, Vertical is 0
  joint_group_positions[wrist_roll_joint_index] = 93*RAD_CONV;

  move_group_interface.setJointValueTarget(joint_group_positions);
  planAndExecuteJoints(move_group_interface, logger);
  RCLCPP_INFO(logger, "Wrist roll corrected");

  geometry_msgs::msg::Pose move_pose = current_pose.pose;
  move_pose.position.y += 0.050;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(move_pose);

  move_pose.position.x += 0.050;
  waypoints.push_back(move_pose);

  move_pose.position.z += 0.050;
  waypoints.push_back(move_pose);

  //Orientation adjustment test, To do next
  tf2::Quaternion q_move(
    move_pose.orientation.x,
    move_pose.orientation.y,
    move_pose.orientation.z,
    move_pose.orientation.w);
  q_move.normalize();
 
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.001;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  
  RCLCPP_INFO(logger, "Cartesian path fraction: %f, waypoints in traj: %zu",
      fraction, trajectory.joint_trajectory.points.size());

  if (fraction > 0.95 && !trajectory.joint_trajectory.points.empty()) {
      move_group_interface.execute(trajectory);
  } else {
      RCLCPP_ERROR(logger, "Cartesian path failed or incomplete (fraction=%.2f) — not executing.", fraction);
  }

  rclcpp::sleep_for(std::chrono::milliseconds(200));
  auto final_pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(logger, "Final pose after Cartesian move: x=%f y=%f z=%f, ",
      final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z);

  tf2::Quaternion q(
    final_pose.pose.orientation.x,
    final_pose.pose.orientation.y,
    final_pose.pose.orientation.z,
    final_pose.pose.orientation.w);
  q.normalize();

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  RCLCPP_INFO(logger, "Final Goal orientation (RPY, degrees): roll=%f pitch=%f yaw=%f",
    roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);


    rclcpp::shutdown();
    spin_thread.join();
    return 0;
  }
