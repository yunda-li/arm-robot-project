#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#define RAD_CONV .017453
using moveit::planning_interface::MoveGroupInterface;


void planAndExecute(MoveGroupInterface &move_group_interface, const rclcpp::Logger &logger){
        auto const [success, plan] = [&move_group_interface](){
        MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success){
        move_group_interface.execute(plan);
    } else {
        RCLCPP_ERROR(logger, "Planning Failed");
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

  auto move_group_interface = MoveGroupInterface(node, "arm_move_group");

  move_group_interface.setGoalPositionTolerance(0.05); 
  move_group_interface.setPlanningTime(15.0);
  move_group_interface.setNumPlanningAttempts(10);

  std::thread spin_thread([&node]() { rclcpp::spin(node); });

  moveit::core::RobotStatePtr real_current_state;
  std::vector<double> joint_values;
  bool got_real_state = false;

  for (int attempt = 0; attempt < 50; ++attempt) {  // up to ~5 seconds
    real_current_state = move_group_interface.getCurrentState();
    const moveit::core::JointModelGroup* jmg_check =
        real_current_state->getJointModelGroup("arm_move_group");
    real_current_state->copyJointGroupPositions(jmg_check, joint_values);

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

    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    move_group_interface.setPositionTarget(.104, .022, .227, "gripper");

    planAndExecute(move_group_interface, logger);

    move_group_interface.setPositionTarget(.081, .233, .227, "gripper");

    planAndExecute(move_group_interface, logger);

    move_group_interface.setPositionTarget(.206, -.206, .146, "gripper");

    planAndExecute(move_group_interface, logger);


    rclcpp::sleep_for(std::chrono::milliseconds(200));
    auto final_pose = move_group_interface.getCurrentPose();
    RCLCPP_INFO(logger, "Final pose after Cartesian move: x=%f y=%f z=%f",
        final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z);


    rclcpp::shutdown();
    spin_thread.join();
    return 0;
  }