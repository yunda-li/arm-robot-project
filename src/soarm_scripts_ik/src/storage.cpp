#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <tf2_eigen/tf2_eigen.hpp>


// void move_robot(const std::shared_ptr<rclcpp::Node> node){
//     auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm_move_group");

//     arm_move_group.setPlanningTime(30.0);
//     arm_move_group.setNumPlanningAttempts(100);
//     arm_move_group.setGoalPositionTolerance(0.5); 
//     arm_move_group.setGoalOrientationTolerance(0.5);

//     kinematics::KinematicsQueryOptions ik_options;
//     ik_options.return_approximate_solution = true; 

//     //Move groups defined in .srdf file, use from <group name=> tag
//     // Set up your own QoS-matched subscriber (proven working above)
//     sensor_msgs::msg::JointState::SharedPtr latest_joint_state;
//     auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
//     auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
//         "joint_states", qos,
//         [&latest_joint_state](const sensor_msgs::msg::JointState::SharedPtr msg) {
//             latest_joint_state = msg;
//         });

//     // Give it a moment to receive the first message
//     rclcpp::sleep_for(std::chrono::milliseconds(2000));

//     if (!latest_joint_state) {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No joint state received!");
//         // return;
//     }

//     // Build a RobotState from the real data
//     moveit::core::RobotStatePtr robot_state = arm_move_group.getCurrentState(1.0); 
//     // ^ still fine to call for the RobotState object/robot model itself,
//     //   just don't trust its joint values — we'll overwrite them:

//     robot_state->setVariablePositions(latest_joint_state->name, latest_joint_state->position);
//     robot_state->update(); // recompute FK

//     // Now FK is correct — get the real pose yourself:
//     const Eigen::Isometry3d& gripper_pose = robot_state->getGlobalLinkTransform("gripper");
//     geometry_msgs::msg::Pose pose_msg = tf2::toMsg(gripper_pose);

//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
//         "Real current pose - Position: (%.3f, %.3f, %.3f)",
//         pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);

//     // Use this corrected state as the actual start state for planning:
//     // arm_move_group.setStartState(*robot_state);

//     rclcpp::sleep_for(std::chrono::milliseconds(500));
//     auto current_state = arm_move_group.getCurrentState(10.0); // 10 sec timeout
//     if (!current_state)
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Still no state after waiting!");
//     else {
//         std::vector<double> joint_values;
//         current_state->copyJointGroupPositions("arm_move_group", joint_values);
//         for (size_t i = 0; i < joint_values.size(); ++i) {
//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint[%zu] = %.4f", i, joint_values[i]);
//         }
// }

//     geometry_msgs::msg::PoseStamped current_pose = arm_move_group.getCurrentPose();

//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
//         "Current pose [frame: %s] - Position: (%.3f, %.3f, %.3f) Orientation: (%.3f, %.3f, %.3f, %.3f)",
//         current_pose.header.frame_id.c_str(),
//         current_pose.pose.position.x,
//         current_pose.pose.position.y,
//         current_pose.pose.position.z,
//         current_pose.pose.orientation.x,
//         current_pose.pose.orientation.y,
//         current_pose.pose.orientation.z,
//         current_pose.pose.orientation.w);

//     geometry_msgs::msg::Pose arm_pose;
//     arm_pose.position.x = 0.104;
//     arm_pose.position.y = 0.022;
//     arm_pose.position.z = 0.257; //+.03
//     arm_pose.orientation.x = -.024;
//     arm_pose.orientation.y = .994;
//     arm_pose.orientation.z = .003;
//     arm_pose.orientation.w = -.110;

    // geometry_msgs::msg::Pose arm_pose;
    // arm_pose.position.x = 0.213;
    // arm_pose.position.y = -0.131;
    // arm_pose.position.z = 0.147;
    // arm_pose.orientation.x = -.441;
    // arm_pose.orientation.y = -.089;
    // arm_pose.orientation.z = .505;
    // arm_pose.orientation.w = -.736;

//     bool arm_within_bounds = arm_move_group.setPoseTarget(arm_pose); 

//     bool ik_found = robot_state->setFromIK(
//         robot_state->getJointModelGroup("arm_move_group"),
//         arm_pose,
//         "gripper",
//         5.0,                                          // timeout
//         moveit::core::GroupStateValidityCallbackFn(),  // no validity callback
//         ik_options);

//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IK found (approx): %s", ik_found ? "YES" : "NO");


//     if (!arm_within_bounds){
//         RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target joint position outside of bounds");
//     //     // return;
//     }

//     moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

//     bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     // if (arm_plan_success)
//         arm_move_group.move();

// }

// int main(int argc, char * argv[]){

//     rclcpp::init(argc, argv);
//     std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("moveit_test");

//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);
//     std::thread spin_thread([&executor]() { executor.spin(); });

//     move_robot(node);

//     executor.cancel();
//     spin_thread.join();
//     rclcpp::shutdown();

//     return 0;
// }