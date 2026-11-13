#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

#define DEG_TO_RAD 0.017453

using moveit::planning_interface::MoveGroupInterface;

class CalibrateServos : public rclcpp::Node{
    public:
        CalibrateServos() : Node("CalibrateServos", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        {            
        }

    //Lifecycle states to check current joint state?
    void init(){
        arm_move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm_move_group");
        gripper_move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "gripper_move_group");

        test();
    }

    private:
        std::shared_ptr<MoveGroupInterface> arm_move_group_;
        std::shared_ptr<MoveGroupInterface> gripper_move_group_;

        void planJointTarget(const std::vector<double> joint_values){
            arm_move_group_->setJointValueTarget(joint_values);

            auto const [success, plan] = [this](){
            MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(arm_move_group_->plan(msg));
            return std::make_pair(ok, msg);
            }();

            if (success){
                arm_move_group_->execute(plan);
            
            }else{
                RCLCPP_ERROR(this->get_logger(), "Joint Target Planning Failed");
            }

        }

        std::vector<double> checkJoints(){
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

            std::vector<double> gripper_value;
            moveit::core::RobotStatePtr gripper_current_state = gripper_move_group_->getCurrentState();
            const moveit::core::JointModelGroup* gripper_check =
                gripper_current_state->getJointModelGroup("gripper_move_group");
            gripper_current_state->copyJointGroupPositions(gripper_check, gripper_value);            

            RCLCPP_INFO_STREAM(this->get_logger(), "Current Gripper Angle: " << gripper_value[0] << ", Gripper Step: " << gripper_value[0] / DEG_TO_RAD);

            return joint_values;

        }

        void test(){
            std::vector<double> jointTarget = {
                0, // 3.7,
                0, // 3.3,
                0, // 2.4,
                0, // 3.4,
                0, // 3.5, 
            };

            RCLCPP_INFO(this->get_logger(), "Target joints: ===============");
            for (size_t i = 0; i < jointTarget.size(); ++i){
                RCLCPP_INFO_STREAM(this->get_logger(), "Joint " << i << ": " << jointTarget[i]);
            }

            planJointTarget(jointTarget);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            auto current_state = checkJoints();

            if (current_state.size() != jointTarget.size()){
                RCLCPP_WARN(this->get_logger(), "Size Mismatch");
            }

            RCLCPP_INFO(this->get_logger(), "Joint Difference: ============");
            for (size_t i = 0; i < current_state.size(); ++i){
                double diff = current_state[i] - jointTarget[i];
                RCLCPP_INFO_STREAM(this->get_logger(), "Joint " << i <<  " Diff: " << diff);


            }

        }

};

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CalibrateServos>();
  std::thread spin_thread([node]() { rclcpp::spin(node); });
  node->init();
  rclcpp::shutdown();
  spin_thread.join();
  return 0;

}