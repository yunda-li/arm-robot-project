#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <variant>
#include <geometry_msgs/msg/pose.hpp>


#define RAD_CONV 0.017453

using moveit::planning_interface::MoveGroupInterface;
using geometry_msgs::msg::Pose;

namespace ik_tests
{
  class PickPlaceIK : public rclcpp::Node{
    public:
      PickPlaceIK()
      : Node("PickPlaceIk",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
      {
      }

      void init(){
        arm_move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm_move_group");
        arm_move_group_->setPlanningTime(5.0);
        arm_move_group_->setNumPlanningAttempts(10);

        test();
      }

    private:
      std::shared_ptr<MoveGroupInterface> arm_move_group_;

      enum class TargetType {targetPose, targetJoints};

      //Add pause_s eventually
      struct ArmTarget{
        TargetType type;
        std::variant<Pose, std::vector<double>> target;
      };
      
      //Some funky paths this way. Maybe a Pre Pre Pick step? Or make PrePick a Joint Value Target, maybe remove the middle HOME
      const std::unordered_map<std::string, ArmTarget> StepMap_ = 
      {
        {"HOME" ,     {TargetType::targetJoints, std::vector<double>{0, 27*RAD_CONV, -49*RAD_CONV, 22*RAD_CONV, 92*RAD_CONV}}},
        {"NEAR-HOME", {TargetType::targetPose, createArmPose(.174, .022, .253, -.498, -.508, .492, .502)}},
        {"PRE-PICK",  {TargetType::targetJoints, std::vector<double>{-69*RAD_CONV, 63*RAD_CONV, -73*RAD_CONV, 39*RAD_CONV, 102*RAD_CONV}}},
        {"PICK",      {TargetType::targetPose, createArmPose(.077, .220, .137, -.012, -.066, .996, .066)}},
        {"PRE-PLACE", {TargetType::targetJoints, std::vector<double>{58*RAD_CONV, 73*RAD_CONV, -86*RAD_CONV, 65*RAD_CONV, 100*RAD_CONV}}},
        {"PLACE",     {TargetType::targetPose, createArmPose(.215, -.224, .055, -.688, -.189, .207, .669)}}
      };

      const std::vector<std::string> StepSequence_ =
      {
        // "HOME",
        // "PRE-PICK",
        // "PICK",
        // "PRE-PICK",
        // "NEAR-HOME",
        // "HOME",
        // "BOX",
        // "HOME",
        "EIGHT",
        // "PRE-PLACE",
        // "PLACE",
        // "PRE-PLACE",
        // "NEAR-HOME",
        // "HOME"
      };

      //Orientation in RViz is XYZW
      static Pose createArmPose(double x, double y, double z, double qx, double qy, double qz, double qw){
        Pose msg;
        msg.position.x = x;
        msg.position.y = y;
        msg.position.z = z;

        msg.orientation.x = qx;
        msg.orientation.y = qy;
        msg.orientation.z = qz;
        msg.orientation.w = qw;

        return msg;
      }

      bool planAndExecutePose(const Pose &target_pose){

        tf2::Quaternion q(
          target_pose.orientation.x,
          target_pose.orientation.y,
          target_pose.orientation.z,
          target_pose.orientation.w);
        q.normalize();

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "Initial Goal orientation (RPY, degrees): roll=%f pitch=%f yaw=%f",
          roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

        //Orientation relaxing, test with setTargetPose() and setTargetPosition()
        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = "gripper";
        ocm.header.frame_id = arm_move_group_->getPlanningFrame();
        ocm.orientation = target_pose.orientation;

        ocm.absolute_x_axis_tolerance = 90*RAD_CONV; //I think Roll might be the most important for keeping gripper aligned, but with correction can be loose
        ocm.absolute_y_axis_tolerance = 30*RAD_CONV;
        ocm.absolute_z_axis_tolerance = 180*RAD_CONV;
        ocm.weight = 1.0;
        
        moveit_msgs::msg::Constraints constraints;
        constraints.orientation_constraints.push_back(ocm);
        arm_move_group_->setPathConstraints(constraints);

        arm_move_group_->setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z, "gripper");
        MoveGroupInterface::Plan plan;
        bool ok = static_cast<bool>(arm_move_group_->plan(plan));

        if (ok){
          arm_move_group_->execute(plan);
          return true;
        }
        else{
          RCLCPP_WARN(this->get_logger(), "Restrained Position Target Failed");
          return false;
        }
      }

      void planAndExecuteJoints(const std::vector<double> joint_values){
        arm_move_group_->setJointValueTarget(joint_values);

        auto const [success, plan] = [this](){
        MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(arm_move_group_->plan(msg));
        return std::make_pair(ok, msg);
        }();

        if (success){
            arm_move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "POSITION Planning Failed");
        }
      }
      
      void fixWristRoll(double angle){
        std::vector<double> joint_group_positions;
        const moveit::core::JointModelGroup *joint_model_group = arm_move_group_->getCurrentState()->getJointModelGroup("arm_move_group");
        arm_move_group_->getCurrentState()->copyJointGroupPositions(joint_model_group, joint_group_positions);

        int wrist_roll_joint_index = 4;
        //Horizontal flat is 93 or -87, Vertical is 0
        joint_group_positions[wrist_roll_joint_index] = angle*RAD_CONV;

        planAndExecuteJoints(joint_group_positions);
        RCLCPP_INFO(this->get_logger(), "Wrist roll corrected");

      }

      void boxDemo(){
        //Move to helper function=========== maybe only needs to be at the beginning?
        moveit::core::RobotStatePtr real_current_state;
        std::vector<double> joint_values;
        bool got_real_state = false;

        for (int attempt = 0; attempt < 50; ++attempt) {  // up to ~5 seconds
          real_current_state = arm_move_group_->getCurrentState();
          const moveit::core::JointModelGroup* jmg_check =
              real_current_state->getJointModelGroup("arm_move_group");
          real_current_state->copyJointGroupPositions(jmg_check, joint_values);

          // Heuristic: real state is very unlikely to be EXACTLY all zero
          bool all_zero = true;
          for (double v : joint_values) {
            if (std::abs(v) > 1e-6){ 
              all_zero = false; 
              break; 
            }
          }
          if (!all_zero) {
            got_real_state = true;
            break;
          }
          rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        if (!got_real_state) {
          RCLCPP_WARN(this->get_logger(), "Current state still reads all-zero after waiting — may be genuine, or monitor issue persists.");
        }

        for (size_t i = 0; i < joint_values.size(); ++i) {
          RCLCPP_INFO(this->get_logger(), "Joint %zu: %f", i, joint_values[i]);
        }
        //===============

        auto current_pose = arm_move_group_->getCurrentPose();

        RCLCPP_INFO(get_logger(), "Start xyz: %.3f %.3f %.3f",
            current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

        geometry_msgs::msg::Pose base_pose = current_pose.pose;
        RCLCPP_INFO(this->get_logger(), "Planning Box Demo");
        std::vector<geometry_msgs::msg::Pose> waypoints;

        //Diagonal which way? Also, why does this go down when the gripper is high, but up when gripper is low?
        base_pose.position.z += 0.050;
        base_pose.position.y += 0.050;
        waypoints.push_back(base_pose);

        base_pose.position.z -= 0.10;
        waypoints.push_back(base_pose);

        base_pose.position.y -= 0.10;
        waypoints.push_back(base_pose);

        base_pose.position.z += 0.10;
        waypoints.push_back(base_pose);

        base_pose.position.y += 0.10;
        waypoints.push_back(base_pose);

        base_pose.position.z -= 0.050;
        base_pose.position.y -= 0.050;
        waypoints.push_back(base_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = arm_move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
        RCLCPP_INFO(this->get_logger(), "Cartesian path fraction: %f, waypoints in traj: %zu",
            fraction, trajectory.joint_trajectory.points.size());

        if (fraction > 0.95 && !trajectory.joint_trajectory.points.empty()) {
            arm_move_group_->execute(trajectory);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cartesian path failed or incomplete (fraction=%.2f) — not executing.", fraction);
        }

      }

      void eightDemo(){
        //Move to helper function=====
        moveit::core::RobotStatePtr real_current_state;
        std::vector<double> joint_values;
        bool got_real_state = false;

        for (int attempt = 0; attempt < 50; ++attempt) {
          real_current_state = arm_move_group_->getCurrentState();
          const moveit::core::JointModelGroup* jmg_check =
              real_current_state->getJointModelGroup("arm_move_group");
          real_current_state->copyJointGroupPositions(jmg_check, joint_values);

          // Heuristic: real state is very unlikely to be EXACTLY all zero
          bool all_zero = true;
          for (double v : joint_values) {
            if (std::abs(v) > 1e-6){ 
              all_zero = false; 
              break; 
            }
          }
          if (!all_zero) {
            got_real_state = true;
            break;
          }
          rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        if (!got_real_state) {
          RCLCPP_WARN(this->get_logger(), "Current state still reads all-zero after waiting — may be genuine, or monitor issue persists.");
        }

        for (size_t i = 0; i < joint_values.size(); ++i) {
          RCLCPP_INFO(this->get_logger(), "Joint %zu: %f", i, joint_values[i]);
        }
        //===============

        const auto start_pose = arm_move_group_->getCurrentPose().pose;
        const double r = 0.05;
        const int steps = 36;
        const double y_center = start_pose.position.y;
        const double z_center = start_pose.position.z + r;
        std::vector<Pose> waypoints;

        for (size_t i = 0; i <= steps; ++i){
          double theta = 2.0 * M_PI * i / steps;
          //Might need to double check diretcions, but Z is vertical (Y) and Y is lateral (X)
          Pose pose = start_pose;
          pose.position.y = y_center + r * std::sin(theta);
          pose.position.z = z_center + r * std::cos(theta);
          waypoints.push_back(pose);
        }

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = arm_move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
        RCLCPP_INFO(this->get_logger(), "Cartesian path fraction: %f, waypoints in traj: %zu",
            fraction, trajectory.joint_trajectory.points.size());

        if (fraction > 0.95 && !trajectory.joint_trajectory.points.empty()) {
            arm_move_group_->execute(trajectory);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cartesian path failed or incomplete (fraction=%.2f) — not executing.", fraction);
        }


      }


      void test(){
        for (auto & step_name : StepSequence_){
          if (step_name == "BOX"){
            boxDemo();
            continue;
          }

          if (step_name == "EIGHT"){
            eightDemo();
            continue;
          }
      
          auto step_it = StepMap_.find(step_name);
          // if (step_it == StepMap_.end()){
          //   RCLCPP_ERROR(this->get_logger(), "Pose not found, skipping to next");
          //   continue;
          // }

          RCLCPP_INFO_STREAM(this->get_logger(), "====STEP PLANNED TO: " << step_it->first << "==========");

          if (step_it->first == "CUSTOM"){
            continue;

          }

          //Change over to get_if eventually
          else if (step_it->second.type == TargetType::targetPose){
            const auto& created_pose = std::get<Pose>(step_it->second.target);
            planAndExecutePose(created_pose);

            auto current_pose = arm_move_group_->getCurrentPose();
            RCLCPP_INFO_STREAM(this->get_logger(), "CURRENT POSITION: " << current_pose.pose.position.x << ", " 
              << current_pose.pose.position.y << ", " << current_pose.pose.position.z);

            tf2::Quaternion q_curr(
              current_pose.pose.orientation.x,
              current_pose.pose.orientation.y,
              current_pose.pose.orientation.z,
              current_pose.pose.orientation.w);
            q_curr.normalize();

            double x, y, z;
            tf2::Matrix3x3(q_curr).getRPY(x,y,z);

            RCLCPP_INFO_STREAM(this->get_logger(), "CURRENT ORIENTATION (RPY): " << x*180/M_PI << ", " << y*180/M_PI << ", " << z*180/M_PI);
          }

          //Change over to get_if eventually
          else if (step_it->second.type == TargetType::targetJoints){
            auto &joints = std::get<std::vector<double>>(step_it->second.target);

            planAndExecuteJoints(joints);
          }

          else{
            RCLCPP_ERROR(this->get_logger(), "ERROR IN STEP PARSING");
            continue;
          }

        }
        
        }
  };
}

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ik_tests::PickPlaceIK>();
  std::thread spin_thread([node]() { rclcpp::spin(node); });
  node->init();
  rclcpp::shutdown();
  spin_thread.join();
  return 0;

}
