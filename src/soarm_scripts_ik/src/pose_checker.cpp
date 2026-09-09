#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <tf2_eigen/tf2_eigen.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true) //IS this what fixed the entire thing?
  );

  auto const logger = rclcpp::get_logger("hello_moveit");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm_move_group");

  rclcpp::sleep_for(std::chrono::seconds(2)); // crude but effective test

  move_group_interface.setGoalPositionTolerance(0.5);
  move_group_interface.setGoalOrientationTolerance(0.5);
  move_group_interface.setNumPlanningAttempts(50);
  move_group_interface.setPlanningTime(30.0);

  // HOME
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
  // auto const target_pose = []{
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

  // //PRE-PLACE
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

  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.050981;
    msg.position.y = 0.230670;
    msg.position.z = 0.154420;

    msg.orientation.x = .13687;
    msg.orientation.y = -.016418;
    msg.orientation.z = .00065949;
    msg.orientation.w = .99045;

    return msg;
  }();  


  {
    auto const& q = target_pose.orientation;
    double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    RCLCPP_INFO(logger, "Quaternion norm: %.6f (should be ~1.0)", norm);
    if (std::abs(norm - 1.0) > 0.01) {
      RCLCPP_WARN(logger, "Quaternion is NOT normalized! This is likely why IK is failing.");
    }

    RCLCPP_INFO(logger, "Planning frame:        %s", move_group_interface.getPlanningFrame().c_str());
    RCLCPP_INFO(logger, "Pose reference frame:   %s", move_group_interface.getPoseReferenceFrame().c_str());
    RCLCPP_INFO(logger, "End effector link:      %s", move_group_interface.getEndEffectorLink().c_str());
  }

  move_group_interface.setStartStateToCurrentState();
  bool ik_ok = move_group_interface.setPoseTarget(target_pose); // uses default EE link + solver
  RCLCPP_INFO(logger, "IK for target pose: %s", ik_ok ? "SUCCESS" : "FAILED");

  move_group_interface.setPoseTarget(target_pose);
  // move_group_interface.setPositionTarget(.104, .022, .217, "gripper");

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
    return 0;
  }
