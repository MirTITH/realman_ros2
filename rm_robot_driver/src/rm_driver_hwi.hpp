#pragma once

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <mutex>

#include "robot_interfaces.hpp"
#include "data_with_lock.hpp"

#include <thread>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rm_ros_interfaces/msg/jointpos.hpp>

using hardware_interface::return_type;

namespace rm_driver
{

using CallbackReturn = hardware_interface::CallbackReturn;

class RmDriverHardwareInterface : public hardware_interface::SystemInterface
{
public:
    ~RmDriverHardwareInterface() override;
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    return_type prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) override;
    return_type perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) override;

protected:
    bool is_active_{false};
    rclcpp::Node::SharedPtr node_;
    std::thread ros_thread_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos>::SharedPtr movej_canfd_cmd_pub_;

    std::vector<std::string> rm_driver_joint_names_;

    // Parameters
    std::string tf_prefix_;
    std::vector<double> joint_lower_limits_;
    std::vector<double> joint_upper_limits_;

    // This is updated by joint state topic callback
    DataWithLock<std::vector<double>> joint_positions_;

    // State interfaces
    RobotInterfaces joint_position_state_;

    // Command interfaces
    RobotInterfaces movej_canfd_;

    double StringToDouble(const std::string &str, double default_value = 0)
    {
        if (str.empty()) {
            return default_value;
        }
        return std::stod(str);
    }

    std::string RemovePrefix(const std::string &str, const std::string &prefix)
    {
        if (str.find(prefix) == 0) {
            return str.substr(prefix.length());
        }
        return str;
    }
};
} // namespace rm_driver