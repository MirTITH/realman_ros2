#include "rm_driver_hwi.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
// #include <fmt/ranges.h>

namespace rm_driver
{

CallbackReturn RmDriverHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    node_ = rclcpp::Node::make_shared(info.name + "_hwi");
    RCLCPP_INFO(node_->get_logger(), "on_init");

    tf_prefix_ = info_.hardware_parameters.at("tf_prefix");

    std::vector<std::string> ros_joint_names;

    for (const auto &joint : info.joints) {
        for (const auto &interface : joint.state_interfaces) {
            if (interface.name == "position") {
                ros_joint_names.push_back(joint.name);
            }
        }
    }

    if (ros_joint_names.size() == 6) {
        RCLCPP_INFO(node_->get_logger(), "Detected 6 joints, using 6 dof mode.");
        rm_driver_joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    } else if (ros_joint_names.size() == 7) {
        RCLCPP_INFO(node_->get_logger(), "Detected 7 joints, using 7 dof mode.");
        rm_driver_joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Unsupported number of joints with position interface: %zu", ros_joint_names.size());
        return CallbackReturn::ERROR;
    }

    // Joint names in hardware info must match the expected joint names
    for (const auto &rm_driver_joint_name : rm_driver_joint_names_) {
        auto exp_ros_joint_name = tf_prefix_ + rm_driver_joint_name;
        if (std::find(ros_joint_names.begin(), ros_joint_names.end(), exp_ros_joint_name) == ros_joint_names.end()) {
            RCLCPP_ERROR(node_->get_logger(), "Joint %s not found in hardware info", exp_ros_joint_name.c_str());
            return CallbackReturn::ERROR;
        }
    }

    // driver_namespace
    auto driver_namespace = info.hardware_parameters.at("driver_namespace");
    RCLCPP_INFO(node_->get_logger(), "driver_namespace: %s", driver_namespace.c_str());

    // Joint state subscription
    joint_position_state_.Init(rm_driver_joint_names_.size(), 0.0);

    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        driver_namespace + "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            const auto joint_num = rm_driver_joint_names_.size();
            std::vector<double> joint_positions(joint_num, std::numeric_limits<double>::quiet_NaN());

            for (size_t i = 0; i < joint_num; ++i) {
                auto it = std::find(msg->name.begin(), msg->name.end(), rm_driver_joint_names_.at(i));
                if (it != msg->name.end()) {
                    size_t index          = std::distance(msg->name.begin(), it);
                    joint_positions.at(i) = msg->position.at(index);
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Joint %s not found in JointState message", rm_driver_joint_names_.at(i).c_str());
                }
            }
            joint_positions_.store(std::move(joint_positions));
        });

    // movej_canfd
    movej_canfd_cmd_pub_ = node_->create_publisher<rm_ros_interfaces::msg::Jointpos>(driver_namespace + "/rm_driver/movej_canfd_cmd", 10);
    auto movej_canfd_div = std::stoi(info.hardware_parameters.at("movej_canfd_div"));
    RCLCPP_INFO(node_->get_logger(), "movej_canfd_div: %d", movej_canfd_div);
    movej_canfd_.Init(rm_driver_joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
    movej_canfd_.SetUpdateRateDivider(movej_canfd_div);
    movej_canfd_.SetUpdateCallback([this](std::vector<double> &data) {
        if (!movej_canfd_.ContainsNaN()) {
            rm_ros_interfaces::msg::Jointpos msg;
            msg.dof    = static_cast<uint8_t>(data.size());
            msg.follow = true;
            msg.joint.resize(data.size());
            std::copy(data.begin(), data.end(), msg.joint.begin());
            movej_canfd_cmd_pub_->publish(msg);
        }
    });
    // movej_canfd_.SetMode(RobotInterfaceMode::ACTIVATE);
    // movej_canfd_.EnableDebug(info.name + "_movej_canfd");

    ros_thread_ = std::thread([this]() {
        RCLCPP_INFO(node_->get_logger(), "ROS thread started");
        rclcpp::spin(node_);
        RCLCPP_INFO(node_->get_logger(), "ROS thread stopped");
    });

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RmDriverHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    const auto joint_num = rm_driver_joint_names_.size();
    for (size_t i = 0; i < joint_num; i++) {
        state_interfaces.emplace_back(tf_prefix_ + rm_driver_joint_names_.at(i), hardware_interface::HW_IF_POSITION, &joint_position_state_.at(i));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RmDriverHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    const auto joint_num = rm_driver_joint_names_.size();
    for (size_t i = 0; i < joint_num; i++) {
        command_interfaces.emplace_back(tf_prefix_ + rm_driver_joint_names_.at(i), hardware_interface::HW_IF_POSITION, &movej_canfd_.at(i));
    }

    return command_interfaces;
}

return_type RmDriverHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    auto joint_positions = joint_positions_.load();
    const auto joint_num = joint_positions.size();

    // Check if the size of joint_positions matches the expected size
    // This may happen if the joint state topic is not received yet
    if (joint_num != joint_position_state_.GetDataLength()) {
        return return_type::OK;
    }
    for (size_t i = 0; i < joint_num; i++) {
        joint_position_state_.at(i) = joint_positions.at(i);
    }
    return return_type::OK;
}

return_type RmDriverHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (is_active_) {
        movej_canfd_.Update();
    }
    return return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RmDriverHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(node_->get_logger(), "on_configure");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RmDriverHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(node_->get_logger(), "on_activate");
    is_active_ = true;
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RmDriverHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(node_->get_logger(), "on_deactivate");
    is_active_ = false;
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RmDriverHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(node_->get_logger(), "on_cleanup");
    // // Shutdown node
    node_->get_node_base_interface()->get_context()->shutdown("RmDriverHardwareInterface on_cleanup");
    ros_thread_.join();
    return CallbackReturn::SUCCESS;
}

return_type RmDriverHardwareInterface::prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
{
    for (const auto &interface : start_interfaces) {
        RCLCPP_INFO(node_->get_logger(), "prepare_command_mode_switch: start interface: %s", interface.c_str());
        if (interface == tf_prefix_ + rm_driver_joint_names_.at(0) + "/" + hardware_interface::HW_IF_POSITION) {
            RCLCPP_INFO(node_->get_logger(), "prepare_command_mode_switch: movej_canfd_.SetMode(RobotInterfaceMode::ACTIVATE)");
            movej_canfd_.SetMode(RobotInterfaceMode::ACTIVATE);
        }
    }
    for (const auto &interface : stop_interfaces) {
        RCLCPP_INFO(node_->get_logger(), "prepare_command_mode_switch: stop interface: %s", interface.c_str());
        if (interface == tf_prefix_ + rm_driver_joint_names_.at(0) + "/" + hardware_interface::HW_IF_POSITION) {
            RCLCPP_INFO(node_->get_logger(), "prepare_command_mode_switch: movej_canfd_.SetMode(RobotInterfaceMode::STOP)");
            movej_canfd_.SetMode(RobotInterfaceMode::STOP);
        }
    }
    // Mode switching is performed here instead of in perform_command_mode_switch,
    // because returning ERROR here can prevent the mode switch,
    // whereas returning ERROR in perform_command_mode_switch will not.
    // auto result = mode_manager_.SwitchMode(start_interfaces, stop_interfaces);
    // return result ? return_type::OK : return_type::ERROR;
    return return_type::OK;
}
return_type RmDriverHardwareInterface::perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
{
    (void)start_interfaces;
    (void)stop_interfaces;
    // logger_.Info("perform_command_mode_switch: start_interfaces: {}, stop_interfaces: {}", start_interfaces, stop_interfaces);
    return return_type::OK;
}
RmDriverHardwareInterface::~RmDriverHardwareInterface()
{
    // logger_.Info("Destructor");
    // If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
    // We therefore need to make sure to actually deactivate the communication
    on_cleanup(rclcpp_lifecycle::State());
}
} // namespace rm_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rm_driver::RmDriverHardwareInterface, hardware_interface::SystemInterface)