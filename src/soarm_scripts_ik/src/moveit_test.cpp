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
      
      //Some funky paths this way. Maybe a Pre Pre Pick step? Or make PrePick a Joint Value Target
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
        "HOME",
        "PRE-PICK",
        "PICK",
        "NEAR-HOME",
        "HOME",
        "BOX",
        "EIGHT",
        "PRE-PLACE",
        "PLACE"
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

      bool planAndExecutePose(MoveGroupInterface &move_group_interface, const Pose &target_pose){

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
        ocm.header.frame_id = move_group_interface.getPlanningFrame();
        ocm.orientation = target_pose.orientation;

        ocm.absolute_x_axis_tolerance = 90*RAD_CONV; //I think Roll might be the most important for keeping gripper aligned, but with correction can be loose
        ocm.absolute_y_axis_tolerance = 45*RAD_CONV;
        ocm.absolute_z_axis_tolerance = 180*RAD_CONV;
        ocm.weight = 1.0;
        
        moveit_msgs::msg::Constraints constraints;
        constraints.orientation_constraints.push_back(ocm);
        move_group_interface.setPathConstraints(constraints);

        move_group_interface.setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z, "gripper");
        MoveGroupInterface::Plan plan;
        bool ok = static_cast<bool>(move_group_interface.plan(plan));

        if (ok){
          move_group_interface.execute(plan);
          return true;
        }
        else{
          RCLCPP_WARN(this->get_logger(), "Restrained Position Target Failed");
          return false;
        }
      }

      void planAndExecuteJoints(MoveGroupInterface &move_group_interface, const std::vector<double> joint_values){
        move_group_interface.setJointValueTarget(joint_values);

        auto const [success, plan] = [&move_group_interface](){
        MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
        }();

        if (success){
            move_group_interface.execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "POSITION Planning Failed");
        }
      }
      
      void fixWristRoll(MoveGroupInterface &move_group_interface, double orientation){


      }

      void boxDemo(MoveGroupInterface &move_group_interface){
        auto current_pose = move_group_interface.getCurrentPose();
        geometry_msgs::msg::Pose base_pose = current_pose.pose;
        RCLCPP_INFO(this->get_logger(), "Planning Box Demo");

        base_pose.position.z += 0.100;
        base_pose.position.y += 0.100;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(base_pose);

        // base_pose.position.x += 0.050;
        // waypoints.push_back(base_pose);

        // base_pose.position.z += 0.050;
        // waypoints.push_back(base_pose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
        RCLCPP_INFO(this->get_logger(), "Cartesian path fraction: %f, waypoints in traj: %zu",
            fraction, trajectory.joint_trajectory.points.size());

        if (fraction > 0.95 && !trajectory.joint_trajectory.points.empty()) {
            move_group_interface.execute(trajectory);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cartesian path failed or incomplete (fraction=%.2f) — not executing.", fraction);
        }

      }

      void eightDemo(MoveGroupInterface &move_group_interface){


      }


      void test(){
        for (auto & step_name : StepSequence_){
          auto step_it = StepMap_.find(step_name);
          // if (step_it == StepMap_.end()){
          //   RCLCPP_ERROR(this->get_logger(), "Pose not found, skipping to next");
          //   continue;
          // }

          RCLCPP_INFO_STREAM(this->get_logger(), "====STEP PLANNED TO: " << step_it->first << "==========");

          if (step_it->first == "CUSTOM"){
            continue;

          }

          else if (step_it->first == "BOX"){
            boxDemo(*arm_move_group_);
          }

          else if (step_it->first == "EIGHT"){
            continue;
            
          }
          //Change over to get_if eventually
          else if (step_it->second.type == TargetType::targetPose){
            const auto& created_pose = std::get<Pose>(step_it->second.target);
            planAndExecutePose(*arm_move_group_, created_pose);

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

            planAndExecuteJoints(*arm_move_group_, joints);
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
