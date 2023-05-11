#ifndef F1TENTH_HARDWARE_INTERFACE__F1TENTH_HARDWARE_INTERFACE_HPP_
#define F1TENTH_HARDWARE_INTERFACE__F1TENTH_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <map>
#include <memory>
#include <string>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using hardware_interface::return_type;


namespace f1tenth_hardware_interface
{
class F1TENTHSystemHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(F1TENTHSystemHardware);

    
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    // Parameters for the F1TENTH simulation
    double hw_start_sec_;
    double hw_stop_sec_;

    // State and command variables
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
};
} // namespace f1tenth_hardware_interface

#endif // F1TENTH_HARDWARE_INTERFACE__F1TENTH_HARDWARE_INTERFACE_HPP_