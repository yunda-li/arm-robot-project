#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp> 
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <urdf/model.h>
#include <chrono> 
#include <memory>
#include <random>
#include <sstream>

using namespace std::chrono_literals;
using namespace std::placeholders;

#define RAD_CONV 0.001533979

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

class JointSweeper : public rclcpp_lifecycle::LifecycleNode
{
    public:     
        JointSweeper(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): 
        rclcpp_lifecycle::LifecycleNode("JointSweeper", options)
        {            
            this->declare_parameter<std::string>("robot_description", "");

            RCLCPP_INFO_STREAM(this->get_logger(), "Lifecycle Node ready for config"); 
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State &) override{
            
            RCLCPP_INFO(this->get_logger(), "Configuring Node");

            pub_ = create_publisher<JointTrajectory>("/arm_controller/joint_trajectory", 1);
            timer_ = create_wall_timer(50ms, std::bind(&JointSweeper::tick, this));

            pullJointLimits();

            
            return CallbackReturn::SUCCESS;

        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State &) override{
            pub_->on_activate();
            homeJoints();

            RCLCPP_INFO(this->get_logger(), "Node ACTIVE, publisher enabled");

            // pullJointLimits();

            sweep_index_ = 0;
            enterState(State::SWEEP_LOWER, move_time_);
            lowerSweep();


            return CallbackReturn::SUCCESS;

        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override{
            RCLCPP_INFO(this->get_logger(), "Node Deactivated");

            //homeJoints without blocking sleeps
            homeJoints();

            pub_->on_deactivate();

            timer_->cancel();
            state_ = State::IDLE;
            sweep_index_ = 0;

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override{
            RCLCPP_INFO(this->get_logger(), "Cleaning up resources");

            timer_.reset();
            pub_.reset();

            joint_data_.clear();

            state_ = State::IDLE;

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override{
            RCLCPP_INFO(this->get_logger(), "Shutting down");

            timer_.reset();
            pub_.reset();

            return CallbackReturn::SUCCESS;
        }



    private: 
        enum class State{
            IDLE,
            SWEEP_LOWER,
            SWEEP_UPPER,
            DONE
        };

        //Starting State
        State state_ = State::IDLE;
        rclcpp::Time state_entered_at_;
        rclcpp::Duration state_dwell_ = rclcpp::Duration::from_seconds(0.0);
        size_t sweep_index_ = 0;

        rclcpp_lifecycle::LifecyclePublisher<JointTrajectory>::SharedPtr pub_;  
        rclcpp::TimerBase::SharedPtr timer_;

        double move_time_ = 2.0;

        const std::vector<std::string> joint_names_ = {
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
                // "gripper_joint"
        };

        // const std::vector<double> joint_homes_ = {
        //         {2500 * RAD_CONV},
        //         {1000 * RAD_CONV},
        //         {2500 * RAD_CONV},
        //         {1300 * RAD_CONV},
        //         {3300 * RAD_CONV},
        //         // "gripper_joint"
        // };

        //Different sim home for collision reasons
        const std::vector<double> joint_homes_ = {
                0,
                1.5,
                -1.5,
                0,
                0,
                // "gripper_joint"
        };

        std::map<std::string, std::pair<double, double>> joint_data_;

        //State handling, record entry time + dwell duration, compare against current time
        void enterState(State next, double dwell_s){
            state_ = next;
            state_entered_at_ = this->get_clock()->now();
            state_dwell_ = rclcpp::Duration::from_seconds(dwell_s);
        }

        //Would be replaced with joint feedback
        bool dwellElapsed(){
            return (this->get_clock()->now() - state_entered_at_) >= state_dwell_;
        }

        //Pull Joint limits from URDF or YAML or robot_description parameter
        void pullJointLimits(){
            std::string urdf_string;

            //2 args, so that whatever is gotten from parameter is automatically stored in 2nd arg's empty variable
            if (!this->get_parameter("robot_description", urdf_string) || urdf_string.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Could not find or read 'robot_description' parameter.");
                return;
            }

            urdf::Model model;
                        
            if (!model.initString(urdf_string)) {
                RCLCPP_ERROR(this->get_logger(), "Couldn't parse URDF");
                return;
            }
            joint_data_.clear();

            for (const auto& name : joint_names_){
                auto joint = model.getJoint(name);
                if (joint && joint->limits) {
                    double lower_limit = joint->limits->lower;
                    double upper_limit = joint->limits->upper;

                    joint_data_[name] = {lower_limit, upper_limit};

                    RCLCPP_INFO_STREAM(this->get_logger(), name << " limits, upper: " << upper_limit << " lower: " << lower_limit);
                }
                else{
                    RCLCPP_WARN(this->get_logger(), "Could not find URDF joints for %s" , name.c_str());
                }
            }
            
        }

        void publishPositions(const std::vector<double> &positions, double duration_s){
            auto message = trajectory_msgs::msg::JointTrajectory();
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = "base_link";
            message.joint_names = joint_names_; 

            auto point = trajectory_msgs::msg::JointTrajectoryPoint();
            point.positions = positions;
            point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
            message.points.push_back(point);

            pub_->publish(message);
        }

        // void startSweep(){
        //     if (joint_data_.empty()) {
        //         RCLCPP_WARN(this->get_logger(), "No Joint Trajectory Provided or Invalid Trajectory");
        //         state_ = State::IDLE;
        //         sweep_index_ = 0;
        //         return;
        //     }
        //     sweep_index_ = 0;
        // }

        void homeJoints(){
            publishPositions(joint_homes_, move_time_);
        }

        void lowerSweep(){
            if (sweep_index_ >= joint_names_.size())
                return;

            const std::string & current_joint = joint_names_[sweep_index_];
            //Don't forget joint_data_ is a map, std::string to vector of doubles
            auto it = joint_data_.find(current_joint);
            if (it == joint_data_.end()){
                RCLCPP_INFO_STREAM(this->get_logger(), "Skipping joint " << current_joint << ", no limit data");
                ++sweep_index_;
                lowerSweep();
                return;
            }

            auto command_vec = joint_homes_;
            command_vec[sweep_index_] = it->second.first;
            publishPositions(command_vec, move_time_);


        }

        void upperSweep(){
            if (sweep_index_ >= joint_names_.size())
                return;

            const std::string & current_joint = joint_names_[sweep_index_];
            auto it = joint_data_.find(current_joint);
            if (it == joint_data_.end()){
                RCLCPP_INFO_STREAM(this->get_logger(), "Skipping joint " << current_joint << ", no limit data");
                ++sweep_index_;
                upperSweep();
                return;
            }

            auto command_vec = joint_homes_;
            command_vec[sweep_index_] = it->second.second;
            publishPositions(command_vec, move_time_);

        }

        void tick(){
            switch(state_){
                case State::IDLE:
                    break;

                // case State::HOMING:
                //     if (dwellElapsed()){
                //         enterState(State::IDLE, 0.0);
                //     }
                //     break;

                case State::SWEEP_LOWER:
                    if (dwellElapsed()){
                        enterState(State::SWEEP_UPPER, move_time_);
                        upperSweep();
                    } 
                    break;

                case State::SWEEP_UPPER:
                    if (dwellElapsed()){
                        ++sweep_index_;
                        if (sweep_index_ >= joint_names_.size()){
                            //Last home when done
                            enterState(State::DONE, 0.0);
                            homeJoints();
                        } else {
                            enterState(State::SWEEP_LOWER, move_time_);
                            lowerSweep();
                        }
                    } 

                    break;

                case State::DONE:
                    break;
            }


        }


        
};


int main(int argc, char* argv[]){ 
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<JointSweeper>(); 
    rclcpp::spin(node->get_node_base_interface()); 
    rclcpp::shutdown(); 
    return 0;
}