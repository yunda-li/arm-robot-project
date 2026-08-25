#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <tf2_eigen/tf2_eigen.hpp>


namespace {

// Prints the arm's current pose (from FK on the live joint state) purely as a
// diagnostic. Not used to seed planning - MoveGroupInterface pulls current
// state itself when plan() is called.
void log_current_pose(const moveit::planning_interface::MoveGroupInterface& arm_move_group,
                       const std::string& tip_link) {
    moveit::core::RobotStatePtr robot_state = arm_move_group.getCurrentState(10.0);
    if (!robot_state) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not get current robot state (timed out)");
        return;
    }

    const Eigen::Isometry3d& tip_pose = robot_state->getGlobalLinkTransform(tip_link);
    geometry_msgs::msg::Pose pose_msg = tf2::toMsg(tip_pose);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "Current pose [%s] - Position: (%.3f, %.3f, %.3f) Orientation: (%.3f, %.3f, %.3f, %.3f)",
        tip_link.c_str(),
        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z,
        pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);

    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions("arm_move_group", joint_values);
    for (size_t i = 0; i < joint_values.size(); ++i) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint[%zu] = %.4f", i, joint_values[i]);
    }
}

// Standalone IK feasibility check. This is diagnostic only - it tells you
// whether *some* solution exists for the target pose, using the same
// pick_ik plugin the planner uses under the hood. It does NOT feed into
// planning; setPoseTarget()/plan() do their own IK internally.
bool check_ik_feasibility(const moveit::planning_interface::MoveGroupInterface& arm_move_group,
                           const geometry_msgs::msg::Pose& target_pose,
                           const std::string& tip_link) {
    moveit::core::RobotStatePtr robot_state = arm_move_group.getCurrentState(10.0);
    if (!robot_state) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not get current robot state for IK check");
        return false;
    }

    kinematics::KinematicsQueryOptions ik_options;
    ik_options.return_approximate_solution = true;

    bool ik_found = robot_state->setFromIK(
        robot_state->getJointModelGroup("arm_move_group"),
        target_pose,
        tip_link,
        5.0,
        moveit::core::GroupStateValidityCallbackFn(),
        ik_options);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "[diagnostic] Standalone IK check for target pose: %s", ik_found ? "SOLVED" : "NO SOLUTION");
    return ik_found;
}

} // namespace

void move_robot(const std::shared_ptr<rclcpp::Node> node) {
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm_move_group");
    const std::string tip_link = "gripper";

    arm_move_group.setPlanningTime(30.0);
    arm_move_group.setNumPlanningAttempts(100);
    arm_move_group.setGoalPositionTolerance(0.01);     // 1 cm
    arm_move_group.setGoalOrientationTolerance(0.05);  // ~3 degrees

    // --- Diagnostics: where are we now? ---
    log_current_pose(arm_move_group, tip_link);

    // --- Target pose ---
    // geometry_msgs::msg::Pose target_pose;
    // target_pose.position.x = 0.104;
    // target_pose.position.y = 0.022;
    // target_pose.position.z = 0.257;
    // target_pose.orientation.x = -0.024;
    // target_pose.orientation.y = 0.994;
    // target_pose.orientation.z = 0.003;
    // target_pose.orientation.w = -0.110;

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.213;
    target_pose.position.y = -0.131;
    target_pose.position.z = 0.147;
    target_pose.orientation.x = -.441;
    target_pose.orientation.y = -.089;
    target_pose.orientation.z = .505;
    target_pose.orientation.w = -.736;

    // Diagnostic only - does not affect the actual plan below.
    check_ik_feasibility(arm_move_group, target_pose, tip_link);

    if (!arm_move_group.setPoseTarget(target_pose, tip_link)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to register pose target with the interface");
        return;
    }

    // --- Plan ---
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    bool plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!plan_success) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planning failed - not executing");
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning succeeded");

    // --- Execute the plan we just validated (not a fresh plan via move()) ---
    auto exec_result = arm_move_group.execute(arm_plan);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution succeeded");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Execution failed with error code: %d", exec_result.val);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("moveit_test");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    move_robot(node);

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();

    return 0;
}