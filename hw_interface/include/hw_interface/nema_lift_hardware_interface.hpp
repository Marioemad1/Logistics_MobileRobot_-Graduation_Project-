#ifndef NEMA_LIFT_HARDWARE_INTERFACE_HPP
#define NEMA_LIFT_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include <libserial/SerialPort.h>

namespace nema_lift_hardware_interface
{

class NemaLiftHardwareInterface : public hardware_interface::SystemInterface
{
public:

    //lifecycle node override
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state);

    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state);
    
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state);

    
    std::vector<hardware_interface::StateInterface>
        export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
        export_command_interfaces() override;

    //system interface override
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;

    hardware_interface::return_type
        read(const rclcpp::Time &time, const rclcpp::Duration & period) override;

    hardware_interface::return_type
        write(const rclcpp::Time &time, const rclcpp::Duration & period) override;

private:

    std::string port_;


    double read_state;
    double write_state;

}; //class

} //namespace



#endif