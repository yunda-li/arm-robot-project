#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;
using geometry_msgs::msg::Pose;

namespace ik_tests
{
  class PickPlaceIK : public rclcpp::Node{
    public:
      //What is the proper format for this?
      PickPlaceServer(const rclcpp::NodeOptions& options = 
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true): Node("PickPlaceIK", options))
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
        {"HOME" ,   {createArmPose()}},
        {"PRE-PICK", {createArmPose()}},
        {"PICK", {createArmPose()}},
        {"PRE-PLACE", {createArmPose()}},
        {"PLACE", {createArmPose()}},

      };

      const std::vector<std::string> PoseSequence_ =
      {
        "HOME",
        "PRE-PICK",
        "PICK",
        "PRE-PLACE",
        "PLACE"
      };


      static const Pose createArmPose(double position_x, double position_y, double position_z, double orient_w){
        Pose msg;
        msg.position.x = position_x;
        msg.position.y = position_y;
        msg.position.z = position_z;

        msg.orientation.w = orient_w;

        return msg;
      }
      
      void test(){

      for (auto & pose_name : PoseSequence_){
        auto pose_it = PoseMap_.find(pose_name);
        if (pose_it == PoseMap_.end()){
          RCLCPP_ERROR(this->get_logger(), "Pose not found, skipping to next");
          continue;
        }

        arm_move_group_.setPoseTarget(pose_it->second);

        auto const [success, plan] = [&move_group_interface]{
          MoveGroupInterface::Plan msg;
          auto const ok = static_cast<bool>(move_group_interface.plan(msg));
          return std::make_pair(ok, msg);
        }();

        // Execute the plan
        if(success) {
          move_group_interface.execute(plan);
          RCLCPP_INFO(this->get_logger(), "Moving to Pose: " << pose_name);
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
  return 0;

}
