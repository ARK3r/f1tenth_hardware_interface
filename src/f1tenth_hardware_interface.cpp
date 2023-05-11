#include "f1tenth_hardware_interface/f1tenth_hardware_interface.hpp"

#include <algorithm>
#include <array>
#include <vector>
#include <chrono>
#include <cmath>
#include <limits>
#include <cstddef>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"


namespace f1tenth_hardware_interface
{

hardware_interface::CallbackReturn F1TENTHSystemHardware::on_init(const hardware_interface::HardwareInfo &info)
{
    RCLCPP_DEBUG(rclcpp::get_logger("F1TENTHSystemHardware"), "on_init");

    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // print info about the hardware interface
    // RCLCPP_DEBUG(rclcpp::get_logger("F1TENTHSystemHardware"), "F1TENTHSystemHardware interface has %d state handles and %d command handles", info_.sensors.size(), info_.actuators.size());

    hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);

    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {

        // log all info about the joint
        RCLCPP_DEBUG(
            rclcpp::get_logger("F1TENTHSystemHardware"),
            "Joint '%s' has %zu command interfaces and %zu state interfaces and they are as follows:",
            joint.name.c_str(), joint.command_interfaces.size(), joint.state_interfaces.size());
        
        for (uint i = 0; i < joint.command_interfaces.size(); i++)
        {
            RCLCPP_DEBUG(
                rclcpp::get_logger("F1TENTHSystemHardware"),
                "Command interface %d: %s", i, joint.command_interfaces[i].name.c_str());
        }

        for (uint i = 0; i < joint.state_interfaces.size(); i++)
        {
            RCLCPP_DEBUG(
                rclcpp::get_logger("F1TENTHSystemHardware"),
                "State interface %d: %s", i, joint.state_interfaces[i].name.c_str());
        }

        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("F1TENTHSystemHardware"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("F1TENTHSystemHardware"),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("F1TENTHSystemHardware"),
                "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("F1TENTHSystemHardware"),
                "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> F1TENTHSystemHardware::export_state_interfaces()
{
    RCLCPP_DEBUG(rclcpp::get_logger("F1TENTHSystemHardware"), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;

    // export state interface
    for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_positions_[i]));
    }

    return state_interfaces;


}

std::vector<hardware_interface::CommandInterface> F1TENTHSystemHardware::export_command_interfaces()
{
    RCLCPP_DEBUG(rclcpp::get_logger("F1TENTHSystemHardware"), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // export command interface
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

    return command_interfaces;
}


hardware_interface::CallbackReturn F1TENTHSystemHardware::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_DEBUG(rclcpp::get_logger("F1TENTHSystemHardware"), "on_activate");

    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("F1TENTHSystemHardware"), "Activating ...please wait...");

    for (int i = 0; i < hw_start_sec_; i++)
    {
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(
            rclcpp::get_logger("F1TENTHSystemHardware"), "%d seconds left...", hw_start_sec_ - i);
    }

    for (auto i = 0u; i < hw_positions_.size(); ++i)
    {
        hw_positions_[i] = 0.0;
        hw_velocities_[i] = 0.0;
        hw_commands_[i] = 0.0;
    }

    RCLCPP_INFO(rclcpp::get_logger("F1TENTHSystemHardware"), "Activated.");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn F1TENTHSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_DEBUG(rclcpp::get_logger("F1TENTHSystemHardware"), "on_deactivate");

    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("F1TENTHSystemHardware"), "Deactivating ...please wait...");

    for (int i = 0; i < hw_stop_sec_; i++)
    {
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(
            rclcpp::get_logger("F1TENTHSystemHardware"), "%d seconds left...", hw_stop_sec_ - i);
    }

    RCLCPP_INFO(rclcpp::get_logger("F1TENTHSystemHardware"), "Deactivated.");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type F1TENTHSystemHardware::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    for (std::size_t i = 0; i < hw_velocities_.size(); ++i)
    {

        hw_positions_[i] += hw_velocities_[i] * period.seconds();
        RCLCPP_DEBUG(
            rclcpp::get_logger("F1TENTHSystemHardware"), 
            "Got position state %.5f and velocity state %.5f for '%s'", 
            hw_positions_[i], hw_velocities_[i],
            info_.joints[i].name.c_str());
    }

}

hardware_interface::return_type F1TENTHSystemHardware::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    
    for (auto i = 0u; i < hw_commands_.size(); ++i)
    {
        hw_velocities_[i] = hw_commands_[i];

        RCLCPP_DEBUG(
            rclcpp::get_logger("F1TENTHSystemHardware"), 
            "Got command %.5f for '%s'", 
            hw_commands_[0], info_.joints[i].name.c_str());
    }

    return hardware_interface::return_type::OK;

}
} // namespace f1tenth_hardware_interface


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(f1tenth_hardware_interface::F1TENTHSystemHardware, hardware_interface::SystemInterface)