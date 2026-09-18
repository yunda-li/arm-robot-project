#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using moveit::planning_interface::MoveGroupInterface;
using geometry_msgs::msg::Pose;

namespace ik_tests
{
  class PickPlaceIK : public rclcpp::Node{
    public:
      // explicit PickPlaceIK(const rclcpp::NodeOptions& options = 
      //   rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)): Node("PickPlaceIK", options)

      PickPlaceIK()
      : Node("PickPlaceIk",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
      {
      }

      void init(){
        arm_move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm_move_group");
        test();
      }

    private:
      std::shared_ptr<MoveGroupInterface> arm_move_group_;

      // struct PoseWithTiming{
      //   Pose arm_pose,
      //   //Gripper Pose eventually
      //   double pause_s
      // };

      const std::unordered_map<std::string, Pose> PoseMap_ = 
      {
        {"HOME" ,   {createArmPose(.104, .022, .217, -.024, .994, .003, -.110)}},
        {"PRE-PICK", {createArmPose(.066, .186, .262, -.081, -.410, .091, -.031)}},
        {"PICK", {createArmPose(.077, .220, .137, -.012, -.066, .996, .066)}},
        {"PRE-PLACE", {createArmPose(.206, -.206, .136, -.691, -.189, .211, .665)}},
        {"PLACE", {createArmPose(.215, -.224, .055, -.688, -.189, .207, .669)}}

      };

      const std::vector<std::string> PoseSequence_ =
      {
        "HOME",
        "PRE-PICK",
        "PICK",
        "HOME",
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

      std::vector<geometry_msgs::msg::Quaternion> generateOrientationCandidates(
        const geometry_msgs::msg::Quaternion &original,
        double angle_step = 0.15)
      {

        std::vector<geometry_msgs::msg::Quaternion> candidates;
        candidates.push_back(original); //First candidate is always the original Orientation

        tf2::Quaternion q_original(original.x, original.y, original.z, original.w);
        q_original.normalize();

        std::vector<tf2::Vector3> axes = {
          tf2::Vector3(1, 0, 0),
          tf2::Vector3(0, 1, 0),
          tf2::Vector3(0, 0, 1)
        };

        std::vector<double> signs = {1.0, -1.0};

        for (const auto& axis: axes){
          for (double sign : signs){
            tf2::Quaternion q_delta(axis, sign * angle_step);
            tf2::Quaternion q_new = q_delta * q_original;
            q_new.normalize();

            geometry_msgs::msg::Quaternion msg;
            msg.x = q_new.x();
            msg.y = q_new.y();
            msg.z = q_new.z();
            msg.w = q_new.w();
            candidates.push_back(msg);

          }

        }

        return candidates;
      }

      bool planAndExecutePose(MoveGroupInterface &move_group_interface,
        const Pose &target_pose){

        auto candidates = generateOrientationCandidates(target_pose.orientation);
        
        for (size_t i = 0; i < candidates.size(); ++i){
          Pose test_pose = target_pose;
          test_pose.orientation = candidates[i];

          move_group_interface.setPoseTarget(test_pose, "gripper");

          MoveGroupInterface::Plan plan;
          bool ok = static_cast<bool>(move_group_interface.plan(plan));

          if (ok){
            move_group_interface.execute(plan);
            return true;
          }
          else{
            RCLCPP_WARN(this->get_logger(), "POSE Candidate #%ld Failed", i);
          }
        }

        RCLCPP_ERROR(this->get_logger(), "Planning failed for all Pose Candidates");
        return false;
      }

      void planAndExecutePosition(MoveGroupInterface &move_group_interface){
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
      
      void test(){

      for (auto & pose_name : PoseSequence_){
        auto pose_it = PoseMap_.find(pose_name);
        if (pose_it == PoseMap_.end()){
          RCLCPP_ERROR(this->get_logger(), "Pose not found, skipping to next");
          continue;
        }

        auto created_pose = pose_it->second;
        double x = created_pose.position.x;
        double y = created_pose.position.y;
        double z = created_pose.position.z;

        // arm_move_group_->setPositionTarget(x,y,z, "gripper");

        // planAndExecutePosition(*arm_move_group_);

        RCLCPP_INFO_STREAM(this->get_logger(), "====POSE PLANNED TO: " << pose_it->first);
        arm_move_group_->setPoseTarget(pose_it->second);

        planAndExecutePose(*arm_move_group_, created_pose);

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
