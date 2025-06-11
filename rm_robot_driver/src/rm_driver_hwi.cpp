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

    for (const auto &joint : info.joints) {
        for (const auto &interface : joint.state_interfaces) {
            if (interface.name == "position") {
                joint_names_.push_back(joint.name);
            }
        }
    }

    // driver_namespace
    auto driver_namespace = info.hardware_parameters.at("driver_namespace");
    RCLCPP_INFO(node_->get_logger(), "driver_namespace: %s", driver_namespace.c_str());

    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        driver_namespace + "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            RCLCPP_INFO(node_->get_logger(), "Received JointState message with %zu joints", msg->name.size());
            std::vector<double> joint_positions(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
                if (it != msg->name.end()) {
                    size_t index       = std::distance(msg->name.begin(), it);
                    joint_positions[i] = msg->position.at(index);
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Joint %s not found in JointState message", joint_names_[i].c_str());
                }
            }
            joint_positions_.store(std::move(joint_positions));
        });

    // movej_canfd
    auto movej_canfd_div = std::stoi(info.hardware_parameters.at("movej_canfd_div"));
    RCLCPP_INFO(node_->get_logger(), "movej_canfd_div: %d", movej_canfd_div);
    movej_canfd_.SetUpdateRateDivider(movej_canfd_div);
    // movej_canfd_.SetUpdateCallback([this](std::vector<double> &data) {
    //     if (!movej_canfd_.ContainsNaN()) {
    //         this->arm_client_.MovejCanfd(data);
    //     }
    // });
    // movej_canfd_.EnableDebug(info.name + "_movej_canfd");

    joint_position_state_.Init(joint_names_.size(), 0.0);
    movej_canfd_.Init(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());

    for (size_t i = 0; i < joint_names_.size(); i++) {
        RCLCPP_INFO(node_->get_logger(), "joint_name: %s", joint_names_.at(i).c_str());
    }

    ros_thread_ = std::thread([this]() {
        RCLCPP_INFO(node_->get_logger(), "Starting ROS thread");
        rclcpp::spin(node_);
        RCLCPP_INFO(node_->get_logger(), "ROS thread stopped");
    });

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RmDriverHardwareInterface::export_state_interfaces()
{
    auto tf_prefix = info_.hardware_parameters.at("tf_prefix");

    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < joint_names_.size(); i++) {
        state_interfaces.emplace_back(joint_names_.at(i), hardware_interface::HW_IF_POSITION, &joint_position_state_.at(i));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RmDriverHardwareInterface::export_command_interfaces()
{
    auto tf_prefix = info_.hardware_parameters.at("tf_prefix");

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < joint_names_.size(); i++) {
        command_interfaces.emplace_back(joint_names_.at(i), hardware_interface::HW_IF_POSITION, &movej_canfd_.at(i));
    }

    return command_interfaces;
}

return_type RmDriverHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    auto joint_positions = joint_positions_.load();
    if (joint_positions.size() != joint_names_.size()) {
        // RCLCPP_ERROR(node_->get_logger(), "Joint positions size mismatch: expected %zu, got %zu", joint_names_.size(), joint_positions.size());
        return return_type::OK;
    }
    for (size_t i = 0; i < joint_names_.size(); i++) {
        joint_position_state_.at(i) = joint_positions.at(i);
    }
    return return_type::OK;
}

return_type RmDriverHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (is_active_) {
        // movej_canfd_.Update();
        // auto start_time = std::chrono::high_resolution_clock::now();
        // movej_follow_.Update();
        // auto movej_follow_duration = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count();
        // logger_.Info("movej_follow_: {} ms", 1000.0 * movej_follow_duration);

        // start_time = std::chrono::high_resolution_clock::now();
        // hand_follow_pos_.Update();
        // auto hand_follow_pos_duration = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count();
        // logger_.Info("hand_follow_pos_: {} ms", 1000.0 * hand_follow_pos_duration);
    }
    return return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RmDriverHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(node_->get_logger(), "on_configure");
    // try {
    //     logger_.Info("Connecting to arm {}:{}", arm_ip_, arm_port_);
    //     arm_client_.Connect(arm_ip_, arm_port_);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //     arm_client_.SetHandForce(100);
    // } catch (const std::exception &e) {
    //     logger_.Error("Failed to connect to arm: {}", e.what());
    // }
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
    }
    for (const auto &interface : stop_interfaces) {
        RCLCPP_INFO(node_->get_logger(), "prepare_command_mode_switch: stop interface: %s", interface.c_str());
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