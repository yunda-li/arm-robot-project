#ifndef PICK_PLACE_HPP_
#define PICK_PLACE_HPP_

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

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PickPlace : public rclcpp_lifecycle::LifecycleNode
{
public:
    PickPlace();
    virtual ~PickPlace();

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
    rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
    // rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    static constexpr double RAD_CONV = 0.001533979;

    const std::vector<std::string> joint_names_;
    const std::vector<double> joint_homes_;
    std::map<std::string, std::pair<double, double>> joint_data_;

    void pullJointLimits();
    void publishPositions();
    void homeJoints();
    void lowerSweep();
    void upperSweep();
    void tick();


};

#endif