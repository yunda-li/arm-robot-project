#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

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
        // "PRE-PICK",
        "PICK",
        // "HOME",
        // "PRE-PLACE",
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
      
      void test(){

      for (auto & pose_name : PoseSequence_){
        auto pose_it = PoseMap_.find(pose_name);
        if (pose_it == PoseMap_.end()){
          RCLCPP_ERROR(this->get_logger(), "Pose not found, skipping to next");
          continue;
        }

        arm_move_group_->setPoseTarget(pose_it->second);

        auto const [success, plan] = [this]{
          MoveGroupInterface::Plan msg;
          auto const ok = static_cast<bool>(arm_move_group_->plan(msg));
          return std::make_pair(ok, msg);
        }();

        // Execute the plan
        if(success) {
          arm_move_group_->execute(plan);
          RCLCPP_INFO_STREAM(this->get_logger(), "Moving to Pose: " << pose_name);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Planning failed!");
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
