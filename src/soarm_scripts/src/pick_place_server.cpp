#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <soarm_msgs/action/soarm_task.hpp>
#include <urdf/model.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <memory>
#include <thread>

using namespace std::chrono_literals;
using namespace std::placeholders;

#define RAD_CONV 0.001533979

using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

namespace soarm_scripts
{
    class PickPlaceServer : public rclcpp::Node{
        public:
            explicit PickPlaceServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("PickPlaceServer", options)
                {
                    
                    this->declare_parameter<std::string>("robot_description", "");
                    //Must share name with Client
                    action_server_ = rclcpp_action::create_server<soarm_msgs::action::SoarmTask>(this, "pick_place", 
                        std::bind(&PickPlaceServer::goalCallback, this, _1, _2), 
                        std::bind(&PickPlaceServer::cancelCallback, this, _1), 
                        std::bind(&PickPlaceServer::acceptCallback, this, _1));
                    
                    arm_pub_ = create_publisher<JointTrajectory>("/arm_controller/joint_trajectory", 1);
                    gripper_pub_ = create_publisher<JointTrajectory>("/gripper_controller/joint_trajectory", 1);

                    pullJointLimits();

                    RCLCPP_INFO(this->get_logger(), "Starting Action Server");

                }

        private:
            rclcpp_action::Server<soarm_msgs::action::SoarmTask>::SharedPtr action_server_;
            rclcpp::Publisher<JointTrajectory>::SharedPtr arm_pub_;
            rclcpp::Publisher<JointTrajectory>::SharedPtr gripper_pub_;
            
            const double arm_move_time_ = 2.0;
            const double gripper_move_time_ = 1.0;
            const std::vector<std::string> joint_names_ = {
                    "shoulder_pan",
                    "shoulder_lift",
                    "elbow_flex",
                    "wrist_flex",
                    "wrist_roll",
            };
            const std::vector<double> joint_homes_ = {
                    0,
                    1.5,
                    -1.5,
                    0,
                    0,
            };

            std::vector<double> gripper_pose_ = {0.0, 0.0};

            std::map<std::string, std::pair<double, double>> joint_limits_;

            //Pull Joint limits from URDF or YAML or robot_description parameter
            void pullJointLimits(){
                std::string urdf_string;

                if (!this->get_parameter("robot_description", urdf_string) || urdf_string.empty()) {
                    RCLCPP_ERROR(this->get_logger(), "Could not find or read 'robot_description' parameter.");
                    return;
                }

                //2 args, so that whatever is gotten from parameter is automatically stored in 2nd arg's empty variable
                if (this->get_parameter("robot_description", urdf_string)){
                    urdf::Model model;
                                
                    if (model.initString(urdf_string)) {
                        joint_limits_.clear();

                        for (const auto& name : joint_names_){
                            auto joint = model.getJoint(name);
                            if (joint && joint->limits) {
                                double lower_limit = joint->limits->lower;
                                double upper_limit = joint->limits->upper;

                                joint_limits_[name] = {lower_limit, upper_limit};

                            }
                            else{
                                RCLCPP_WARN(this->get_logger(), "Could not find URDF joints for %s" , name.c_str());
                            }
                        }
                    }
                }    
            }

            void publishPositions(const std::vector<double> &positions, double duration_s, const std::string &log_message){ 
                auto message = trajectory_msgs::msg::JointTrajectory();
                message.header.stamp = this->get_clock()->now();
                message.header.frame_id = "base_link";
                message.joint_names = joint_names_; 

                auto point = trajectory_msgs::msg::JointTrajectoryPoint();
                point.positions = positions;
                point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
                message.points.push_back(point);

                RCLCPP_INFO_STREAM(this->get_logger(), "Published Trajectory Message: " << log_message);
                arm_pub_->publish(message);
            }

            void homeJoints(){
                publishPositions(joint_homes_, arm_move_time_, "HOMING");
            }

            void setGripper(const std::vector<double> &positions, double duration_s, const std::string log_message){
                auto message = trajectory_msgs::msg::JointTrajectory();
                message.header.stamp = this->get_clock()->now();
                message.header.frame_id = "base_link";
                message.joint_names = {"gripper_joint"}; 

                auto point = trajectory_msgs::msg::JointTrajectoryPoint();
                point.positions = positions;
                point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
                message.points.push_back(point);

                RCLCPP_INFO_STREAM(this->get_logger(), "Gripper State: " << log_message);
                gripper_pub_->publish(message);
            }


            //Often validation goes in goalCallback
            rclcpp_action::GoalResponse goalCallback(
                const rclcpp_action::GoalUUID& uuid,
                std::shared_ptr<const soarm_msgs::action::SoarmTask::Goal> goal) {

                (void)uuid;

                if (goal->joint_positions.size() != joint_names_.size()) {
                    RCLCPP_WARN(this->get_logger(), "Rejecting goal '%s': expected %zu joint values, got %zu",
                                goal->pose_name.c_str(), joint_names_.size(), goal->joint_positions.size());
                    return rclcpp_action::GoalResponse::REJECT;
                }

                for (size_t i = 0; i < joint_names_.size(); ++i) {
                    const auto& joint = joint_names_[i];
                    auto it = joint_limits_.find(joint);

                    if (it == joint_limits_.end()) {
                        RCLCPP_WARN(this->get_logger(), "Rejecting goal: unknown joint '%s'", joint.c_str());
                        return rclcpp_action::GoalResponse::REJECT;
                    }

                    double value = goal->joint_positions[i];
                    const auto& limits = it->second;
                    if (value < limits.first || value > limits.second) {
                        RCLCPP_WARN(this->get_logger(), "Rejecting goal '%s': joint '%s' value %.3f out of range [%.3f, %.3f]",
                                    goal->pose_name.c_str(), joint.c_str(), value, limits.first, limits.second);
                        return rclcpp_action::GoalResponse::REJECT;
                    }
                }

                RCLCPP_INFO(this->get_logger(), "Goal '%s' accepted", goal->pose_name.c_str());
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            void acceptCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<soarm_msgs::action::SoarmTask>> goal_handle){
                std::thread{std::bind(&PickPlaceServer::execute,this, _1), goal_handle}.detach();
            }
            
            void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<soarm_msgs::action::SoarmTask>> goal_handle) {
                RCLCPP_INFO(this->get_logger(), "Executing Goal");

                //convert to include move time as part of goal?
                const auto goal = goal_handle->get_goal();
                auto feedback = std::make_shared<soarm_msgs::action::SoarmTask::Feedback>();
                auto result = std::make_shared<soarm_msgs::action::SoarmTask::Result>();

                rclcpp::Rate loop_rate(50ms);
                rclcpp::Time move_start = this->get_clock()->now();
                publishPositions(goal->joint_positions, arm_move_time_, goal->pose_name);
                setGripper(goal->gripper_position, gripper_move_time_, goal->gripper_state);

                //LOOP RATE GOVERNS HOW OFTEN TO CHECK FOR CANCELLATION AND FEEDBACK
                while (rclcpp::ok()){
                    if (goal_handle->is_canceling()){
                        result->success = false;
                        goal_handle->canceled(result);
                        return;
                    }

                    //One day may need to convert to position feedback?

                    double elapsed = (this->get_clock()->now() - move_start).seconds();
                    float progress = static_cast<float>(std::min(elapsed / arm_move_time_, 1.0)); //Maybe change to see time past elapsed?
                    feedback->progress = 100 * progress;
                    goal_handle->publish_feedback(feedback);

                    if (progress >= 1.0)
                        break;

                    loop_rate.sleep();
                }

                result->success = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
                }

            rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<soarm_msgs::action::SoarmTask>> goal_handle){
                RCLCPP_INFO(this->get_logger(), "Request to cancel goal");
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            }
    };
}

//Macro to simplify running action servers
RCLCPP_COMPONENTS_REGISTER_NODE(soarm_scripts::PickPlaceServer);