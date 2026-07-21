#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <soarm_msgs/action/soarm_task.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <memory>

using namespace std::chrono_literals;
using namespace std::placeholders;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

namespace soarm_scripts
{
class PickPlaceClient : public rclcpp_lifecycle::LifecycleNode
{
    public:
        PickPlaceClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): 
        rclcpp_lifecycle::LifecycleNode("PickPlaceClient", options){
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State &) override{
            
            RCLCPP_INFO(this->get_logger(), "Configuring Node");
            //Must share name with server
            client_ = rclcpp_action::create_client<soarm_msgs::action::SoarmTask>(this, "pick_place");
            return CallbackReturn::SUCCESS;

        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State &) override{
            RCLCPP_INFO(this->get_logger(), "Node ACTIVE, sending first goal");
            seq_index_ = 0;
            advance();
            return CallbackReturn::SUCCESS;

        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override{
            RCLCPP_INFO(this->get_logger(), "Node Deactivated");

            if (pause_timer_){
                pause_timer_->cancel();
            }

            //Send async_goal_cancel

        // if (current_goal_handle_) {
        //     auto cancel_future = client_->async_cancel_goal(
        //         current_goal_handle_,
        //         [this](rclcpp_action::Client<soarm_msgs::action::SoarmTask>::CancelResponse::SharedPtr response) {
        //             if (response->return_code == 0) { //ERROR_NONE == 0
        //                 RCLCPP_INFO(this->get_logger(), "Goal cancel request accepted");
        //             } else {
        //                 RCLCPP_WARN(this->get_logger(), "Goal cancel request failed, code: %d", response->return_code);
        //             }
        //         });
        //     current_goal_handle_.reset();
        // }           

            seq_index_ = 0;
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override{
            RCLCPP_INFO(this->get_logger(), "Node Cleanup");
            client_.reset();
            return CallbackReturn::SUCCESS;

        }


    private:
        rclcpp_action::Client<soarm_msgs::action::SoarmTask>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr pause_timer_;
        // rclcpp_action::ClientGoalHandle<soarm_msgs::action::SoarmTask>::SharedPtr current_goal_handle_;

        //Pick Coords: (.228, .326), Place Coords: (.27, .305)
        std::map<std::string, std::vector<double>> arm_pose_table_ = {
            {"HOME", {0,        1.5,    -1.5,   0,      0}},
            {"PRE-PICK", {-1,   2.3,    -1.3,   -1,     -1.6}},
            {"PICK", {-1,       2.6,    -1.5,   -1.1,   -1.6}},
            {"PRE-PLACE", {1,   2.2,    -1,     -1.1,   1.6}},
            {"PLACE", {1,       2.6,    -1.4,   -1.2,   1.6}}
        };

        std::map<std::string, std::vector<double>> gripper_pose_table_ = {
            {"OPEN", {1.5}},
            {"CLOSED", {0}}
        };

        struct SequenceStep{
            std::string arm_pose;
            std::string gripper_pose;
            double pause_s = 0.0;
        };

        std::vector<SequenceStep> pick_place_sequence_ = {
            {"HOME",        "CLOSED",     0.0},
            {"PRE-PICK",    "OPEN",     0.0},
            {"PICK",        "OPEN",     0.0},
            {"PICK",        "CLOSED",     0.0},
            {"PRE-PICK",    "CLOSED",     0.0},
            {"HOME",        "CLOSED",     0.0},
            {"PRE-PLACE",    "CLOSED",     0.0},
            {"PLACE",       "CLOSED",     0.0},
            {"PLACE",       "OPEN",     0.0},
            {"PRE-PLACE",    "OPEN",     0.0},
            {"HOME",        "CLOSED",     0.0},
        };

        size_t seq_index_ = 0;

        //May need to modify in case timer cancels too fast
        void pauseThen(double seconds, std::function<void()> then) {
            pause_timer_ = this->create_wall_timer(std::chrono::duration<double>(seconds), [this, then]() {
                    pause_timer_->cancel();
                    then();
                });
        }

        void sendStepGoal(SequenceStep &step){
            //May be blocking 
            // if(!client_->wait_for_action_server())
            // {
            //     RCLCPP_ERROR(this->get_logger(), "Action Server not available");
            //     return;
            // }

            auto arm_it = arm_pose_table_.find(step.arm_pose);
            if (arm_it == arm_pose_table_.end()){
                RCLCPP_FATAL(this->get_logger(), "Invalid Arm Pose");
                rclcpp::shutdown();
                return;
            }

            auto gripper_it = gripper_pose_table_.find(step.gripper_pose);
            if (gripper_it == gripper_pose_table_.end()){
                RCLCPP_FATAL(this->get_logger(), "Invalid Gripper Pose");
                rclcpp::shutdown();
                return;
            }

            auto goal_msg = soarm_msgs::action::SoarmTask::Goal();
            goal_msg.pose_name = step.arm_pose;
            goal_msg.gripper_state = step.gripper_pose;
            goal_msg.joint_positions = arm_it->second;
            goal_msg.gripper_position = gripper_it->second;

            //Callbacks defined outside of create_ function args, allows customization for each
            //separate request, only used for actions and not services
            auto send_goal_options = rclcpp_action::Client<soarm_msgs::action::SoarmTask>::SendGoalOptions();
            send_goal_options.goal_response_callback    = std::bind(&PickPlaceClient::goalCallback, this, _1);
            send_goal_options.feedback_callback         = std::bind(&PickPlaceClient::feedbackCallback, this, _1, _2);
            send_goal_options.result_callback           = std::bind(&PickPlaceClient::resultCallback, this, _1);

            client_->async_send_goal(goal_msg, send_goal_options);
        }

        void advance(){
            if (seq_index_ >= pick_place_sequence_.size()){
                RCLCPP_INFO(this->get_logger(), "Sequence Complete");
                return;
            }

            SequenceStep step = pick_place_sequence_[seq_index_];
            RCLCPP_INFO_STREAM(this->get_logger(), "Sending Step: " << step.arm_pose << ", " << step.gripper_pose << " with pause: " << step.pause_s);

            sendStepGoal(step);
        }

        void goalCallback(const rclcpp_action::ClientGoalHandle<soarm_msgs::action::SoarmTask>::SharedPtr& goal_handle){
            if(!goal_handle){
                RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }

            return;
        }

        void feedbackCallback(rclcpp_action::ClientGoalHandle<soarm_msgs::action::SoarmTask>::SharedPtr goal_handle, 
            std::shared_ptr<const soarm_msgs::action::SoarmTask::Feedback> feedback)
        {
            (void)goal_handle;
            RCLCPP_INFO_STREAM(this->get_logger(), "Current move Progress: " << feedback->progress << "%");
            return;
        }

        void resultCallback(const rclcpp_action::ClientGoalHandle<soarm_msgs::action::SoarmTask>::WrappedResult& result){
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED){
                RCLCPP_WARN(this->get_logger(), "Goal Canceled or Failed");
                return;
            }
            
            const auto& step = pick_place_sequence_[seq_index_];
            seq_index_++;
            if (step.pause_s > 0.0){
                pauseThen(step.pause_s, [this]() {advance();});
            }
            else {
                advance();
            }
        }
    

};
}


RCLCPP_COMPONENTS_REGISTER_NODE(soarm_scripts::PickPlaceClient);