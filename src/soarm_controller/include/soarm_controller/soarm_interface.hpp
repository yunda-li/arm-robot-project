#ifndef SOARM_INTERFACE_H
#define SOARM_INTERFACE_H

//Need if not defined and define in case multiple uses in a project, prevents duplicates

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>

#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <vector>
#include <string>

//Goal is to redefine functions in these inherited classes, away from previous definitions. 
//Drivers act as translator between OS and hardware

namespace arduinobot_controller
{
//Save a lot of typing effort
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

//New inherit, hardware_interface::SystemInterface
class ArduinobotInterface : public hardware_interface::SystemInterface
{
public:
  //Destructor for safety, makes sure motors stop when program stops
  //Virtual keyword "reaches" through pointer to interact with object that pointer references
  ArduinobotInterface();
  virtual ~ArduinobotInterface();

  // Redefining state transitions from rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  //Must follow Lifecycle node format, important for safety to prevent unplanned motor behavior
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // Redefining functions from inherited class hardware_interface::SystemInterface for 
  //configuring driver bhavior and communication
  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  //Needed private variables like Serial Port, various vectors
  LibSerial::SerialPort arduino_;
  std::string port_;
  std::vector<double> position_commands_;
  std::vector<double> prev_position_commands_;
  std::vector<double> position_states_;
};
}  // namespace arduinobot_controller


#endif  // SOARM_INTERFACE_H