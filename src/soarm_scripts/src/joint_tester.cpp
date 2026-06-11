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

class JointTester : public rclcpp::Node
{
    public:     
        JointTester(): Node("JointTester")
        {            
            this->declare_parameter<std::string>("robot_description", "");

            pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 1); 
            timer_ = create_wall_timer(5s, std::bind(&JointTester::timerCallback, this)); 

            RCLCPP_INFO_STREAM(get_logger(), "Publishing at .2 hz"); 

            initKDL();
        }


    private: 
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;  
        rclcpp::TimerBase::SharedPtr timer_;

        std::random_device rd_;
        std::default_random_engine re_{rd_()};

        std::vector<std::string> joint_names_ = {
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
                // "gripper_joint"
            };
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

                    for (const auto& name : joint_names_){
                        auto joint = model.getJoint(name);
                        if (joint && joint->limits) {
                            double lower_limit = joint->limits->lower;
                            double upper_limit = joint->limits->upper;

                            double random_joint_position = randomJointPosition(lower_limit, upper_limit);
                            joint_angles_.push_back(random_joint_position);

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
            //trajectory_msgs::msg::JointTrajectory example:
            //   header: {
            //     stamp: {sec: 0, nanosec: 0},
            //     frame_id: 'base_link'
            //   },
            //   joint_names: ['shoulder_pan'],
            //   points: [
            //     {
            //       positions: [1.0],
            //       time_from_start: {sec: 2, nanosec: 0}
            //     }
            //   ]
            // }"

            if (!joint_angles_.empty()){
                auto message = trajectory_msgs::msg::JointTrajectory();

                message.header.stamp = this->get_clock()->now();
                // message.header.stamp = rclcpp::Time(0,0);

                message.header.frame_id = "base_link";

                message.joint_names = joint_names_; //{"shoulder_pan", "shoulder_lift", ...}

                auto point = trajectory_msgs::msg::JointTrajectoryPoint();

                point.positions = joint_angles_;
                // point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                point.time_from_start = rclcpp::Duration::from_seconds(4.0);

                message.points.push_back(point);

                pub_->publish(message);

            }

            else{
                RCLCPP_WARN(this->get_logger(), "No Joint Trajectory Provided or Invalid Trajectory");
            }
        }


        void timerCallback(){
            pullJointLimits();

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