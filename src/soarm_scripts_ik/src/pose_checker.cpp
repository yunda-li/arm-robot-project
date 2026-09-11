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

  // //PRE-PICK
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.0814;
    msg.position.y = 0.2331;
    msg.position.z = 0.2168;

    msg.orientation.x = .3863;
    msg.orientation.y = -.0007;
    msg.orientation.z = -.2080;
    msg.orientation.w = .8986;

    return msg;
  }();

  // PRE-PLACE
  // auto const target_pose = []{
  //   geometry_msgs::msg::Pose msg;
  //   msg.position.x = 0.206;
  //   msg.position.y = -0.206;
  //   msg.position.z = 0.136;

  //   msg.orientation.x = -.691;
  //   msg.orientation.y = -.189;
  //   msg.orientation.z = .211;
  //   msg.orientation.w = .665;

  //   return msg;
  // }();  

  // auto const target_pose = []{
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

  auto current_state = move_group_interface.getCurrentState();
  const moveit::core::JointModelGroup* jmg =
      real_current_state->getJointModelGroup("arm_move_group");

  // bool found_ik = real_current_state->setFromIK(jmg, target_pose, "gripper", 0.1);
  // RCLCPP_INFO(logger, "Real IK solve: %s", found_ik ? "SUCCESS" : "FAILED");

  moveit::core::RobotState confirmed_start_state = *real_current_state;

  bool found_ik = false;
  for (int i = 0; i < 20 && !found_ik; ++i) {
    real_current_state->setToRandomPositions(jmg);
    found_ik = real_current_state->setFromIK(jmg, target_pose, "gripper", 0.2);
  }
  RCLCPP_INFO(logger, "IK with random restarts: %s", found_ik ? "SUCCESS" : "FAILED");

  move_group_interface.setStartState(confirmed_start_state); 
  move_group_interface.setGoalPositionTolerance(0.05); 
  move_group_interface.setPositionTarget(.292, .022, .252, "gripper");
  // move_group_interface.setPoseTarget(target_pose);

  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();


  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
  }
