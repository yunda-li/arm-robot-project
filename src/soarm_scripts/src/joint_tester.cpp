#include <rclcpp/rclcpp.hpp> 
#include <std_msgs/msg/string.hpp> 
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <chrono> 
#include <memory>
#include <random>
#include <sstream>

using namespace std::chrono_literals;
using namespace std::placeholders;

#define RAD_CONV 0.001533979

class JointTester : public rclcpp::Node
{
    public:     
        JointTester(): Node("JointTester")
        {            
            this->declare_parameter<std::string>("robot_description", "");

            pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 1); 
            timer_ = create_wall_timer(52s, std::bind(&JointTester::timerCallback, this)); 

            RCLCPP_INFO_STREAM(get_logger(), "Publishing every 35s"); 

            initKDL();
        }


    private: 
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;  
        rclcpp::TimerBase::SharedPtr timer_;

        std::random_device rd_;
        std::default_random_engine re_{rd_()};

        const std::vector<std::string> joint_names_ = {
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
                // "gripper_joint"
        };

        const std::unordered_map<std::string, double> joint_homes_ = {
                {"shoulder_pan", 2500 * RAD_CONV},
                {"shoulder_lift", 1000 * RAD_CONV},
                {"elbow_flex", 2500 * RAD_CONV},
                {"wrist_flex", 1300 * RAD_CONV},
                {"wrist_roll", 3300 * RAD_CONV},
                // "gripper_joint"
        };

        std::map<std::string, std::pair<double, double>> joint_data_;
        std::vector<double> joint_angles_;
        std::vector<KDL::Vector> joint_positions_;

        KDL::Tree kdl_tree_;
        KDL::Chain kdl_chain_;
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

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
                    joint_angles_.clear();
                    joint_data_.clear();

                    for (const auto& name : joint_names_){
                        auto joint = model.getJoint(name);
                        if (joint && joint->limits) {
                            double lower_limit = joint->limits->lower;
                            double upper_limit = joint->limits->upper;

                            double random_joint_position = randomJointPosition(lower_limit, upper_limit);
                            joint_angles_.push_back(random_joint_position);

                            joint_data_[name] = {lower_limit, upper_limit};

                            RCLCPP_INFO_STREAM(this->get_logger(), name << " limits, upper: " << upper_limit << " lower: " << lower_limit
                                                                << "\n random joint position: " << random_joint_position);
                        }
                        else{
                            RCLCPP_WARN(this->get_logger(), "Could not find URDF joints for %s" , name.c_str());
                        }
                    }
                }
            }

            std::stringstream ss;
            ss << "[ ";
            for (const auto& pos : joint_angles_) {
                ss << pos << " ";
            }
            ss << "]";

            RCLCPP_INFO_STREAM(this->get_logger(), "Final generated joint_positions_: " << ss.str());        
        }

        double randomJointPosition(double lowerBound, double upperBound){
            std::uniform_real_distribution<double> unif(lowerBound, upperBound);
            return unif(re_);

        }

        void initKDL() {
            std::string urdf_string;

            if (!this->get_parameter("robot_description", urdf_string) || urdf_string.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to retrieve 'robot_description' parameter, or it is empty.");
                return;
            }

            if (!kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse KDL tree from robot_description.");
                return;
            }

            if (!kdl_tree_.getChain("base", "gripper", kdl_chain_)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to extract chain.");
                return;
            }

            fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
            RCLCPP_INFO(this->get_logger(), "KDL Kinematics successfully initialized with %d joints.", kdl_chain_.getNrOfJoints());
        }
        
        bool validateTrajectory(){
            //Map joint positions to KDL jntArray, static array
            KDL::JntArray joint_array(kdl_chain_.getNrOfJoints());

            for (size_t i = 0; (i < joint_angles_.size()) && (i < joint_array.rows()); ++i){
                joint_array(i) = joint_angles_[i];
            }

            joint_positions_.clear();
            bool all_joints_safe = true;
            double ground_limit = 0.07;

            for (size_t i = 1; i <= kdl_chain_.getNrOfSegments(); ++i){
                KDL::Frame segment_frame;

                //JntToCart takes (input{joint angles}, output(KDL::Frame object, will contain position as .p as 3x1 vector and
                //orientation as .M 3x3 Rotation Matrix, i + 1 for link indices since is 0 indexed))
                int status = fk_solver_->JntToCart(joint_array, segment_frame, i);

                if (status >= 0){
                    joint_positions_.push_back(segment_frame.p);
                    RCLCPP_INFO(this->get_logger(), "Segment %zu Z Coord: %f", i, segment_frame.p.z());
                        if (segment_frame.p.z() < ground_limit && i > 2) {
                            RCLCPP_WARN(this->get_logger(), "Collision Danger! Segment %zu is too close to the ground (Z: %f)", i, segment_frame.p.z());
                            all_joints_safe = false;
                        }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "FK failed for segment %zu", i);
                    return false;
                    }

                }
            return all_joints_safe;
        }

        void publishTrajectory(){
            if (!joint_angles_.empty()){
                auto message = trajectory_msgs::msg::JointTrajectory();
                message.header.stamp = this->get_clock()->now();
                message.header.frame_id = "base_link";
                message.joint_names = joint_names_; 

                auto point = trajectory_msgs::msg::JointTrajectoryPoint();
                point.positions = joint_angles_;
                point.time_from_start = rclcpp::Duration::from_seconds(4.0);
                message.points.push_back(point);

                pub_->publish(message);

            }

            else{
                RCLCPP_WARN(this->get_logger(), "No Joint Trajectory Provided or Invalid Trajectory");
            }
        }

        void homeJoints(){
            std::vector<double> home_vec;
                for (const auto& joint : joint_names_){
                    if (joint_homes_.find(joint) != joint_homes_.end()){
                        home_vec.push_back(joint_homes_.at(joint));
                    } else {
                        home_vec.push_back(0.0);
                    }    
                }

                auto message = trajectory_msgs::msg::JointTrajectory();
                message.header.stamp = this->get_clock()->now();
                message.header.frame_id = "base_link";
                message.joint_names = joint_names_;

                auto point = trajectory_msgs::msg::JointTrajectoryPoint();
                std::vector<double> command_vec = home_vec; 

                point.positions = command_vec;
                point.time_from_start = rclcpp::Duration::from_seconds(2.0); 
                message.points.push_back(point);

                RCLCPP_INFO_STREAM(this->get_logger(), "======Homing=========");
                pub_->publish(message);

                rclcpp::Rate(0.5).sleep();


        }

        void jointSweep(){
            if (!joint_data_.empty()){
                std::vector<double> home_vec;
                for (const auto& joint : joint_names_){
                    if (joint_homes_.find(joint) != joint_homes_.end()){
                        home_vec.push_back(joint_homes_.at(joint));
                    } else {
                        home_vec.push_back(0.0);
                    }    
                }

                for (size_t i = 0; i < joint_names_.size(); ++i){
                    std::string current_joint = joint_names_[i];

                    if (joint_data_.find(current_joint) == joint_data_.end()) {
                        RCLCPP_WARN_STREAM(this->get_logger(), "Skipping " << current_joint << ", no limit data.");
                        continue;
                    }

                    double lower_limit = joint_data_[current_joint].first;
                    double upper_limit = joint_data_[current_joint].second;

                    {
                        auto message = trajectory_msgs::msg::JointTrajectory();
                        message.header.stamp = this->get_clock()->now();
                        message.header.frame_id = "base_link";
                        message.joint_names = joint_names_;

                        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
                        std::vector<double> command_vec = home_vec; 
                        command_vec[i] = lower_limit; 

                        point.positions = command_vec;
                        point.time_from_start = rclcpp::Duration::from_seconds(2.0); 
                        message.points.push_back(point);

                        RCLCPP_INFO_STREAM(this->get_logger(), "Moving " << current_joint << " to LOWER limit: " << lower_limit);
                        pub_->publish(message);

                        rclcpp::Rate(0.5).sleep();

                    }

                    homeJoints();
                    //Publish Upper Bound
                    {
                        auto message = trajectory_msgs::msg::JointTrajectory();
                        message.header.stamp = this->get_clock()->now();
                        message.header.frame_id = "base_link";
                        message.joint_names = joint_names_;

                        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
                        std::vector<double> command_vec = home_vec; 
                        command_vec[i] = upper_limit; 

                        point.positions = command_vec;
                        point.time_from_start = rclcpp::Duration::from_seconds(2.0); 
                        message.points.push_back(point);

                        RCLCPP_INFO_STREAM(this->get_logger(), "Moving " << current_joint << " to UPPER limit: " << upper_limit);
                        pub_->publish(message);

                        rclcpp::Rate(0.5).sleep();


                    }

                    homeJoints();

                }
            }
            else{
                RCLCPP_WARN(this->get_logger(), "No Joint Trajectory Provided or Invalid Trajectory");
            }

        }

        bool flag_ = true;

        void simpleSweep(){
            {
                auto message = trajectory_msgs::msg::JointTrajectory();
                message.header.stamp = this->get_clock()->now();
                message.header.frame_id = "base_link";
                message.joint_names = joint_names_;

                std::vector<double> home_vec;
                for (const auto& joint : joint_names_){
                    if (joint_homes_.find(joint) != joint_homes_.end()){
                        home_vec.push_back(joint_homes_.at(joint));
                    } else {
                        home_vec.push_back(0.0);
                    }    
                }

                auto point = trajectory_msgs::msg::JointTrajectoryPoint();

                if (flag_){
                    home_vec[3] = 1200*RAD_CONV;
                    point.positions = home_vec;
                    RCLCPP_INFO_STREAM(this->get_logger(), "Sending Command to Position 1200");
                } else {
                    home_vec[3] = 3200*RAD_CONV;
                    point.positions = home_vec;
                    RCLCPP_INFO_STREAM(this->get_logger(), "Sending Command to Position 3200");
                }

                point.time_from_start = rclcpp::Duration::from_seconds(2.0); 
                message.points.push_back(point);

                pub_->publish(message);

                flag_ = !flag_;

            }

        }

        void sleepTimerCallback(){
            RCLCPP_INFO(this->get_logger(), "...Sleeping...");
        }

        void timerCallback(){
            pullJointLimits();
            jointSweep();

            if(validateTrajectory()){
                publishTrajectory();
            } else {
                RCLCPP_INFO(this->get_logger(), "Waiting for next Trajectory");
            }
        }

        
};


int main(int argc, char* argv[]){ 
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<JointTester>(); 
    rclcpp::spin(node); 
    rclcpp::shutdown(); 
    return 0;
}