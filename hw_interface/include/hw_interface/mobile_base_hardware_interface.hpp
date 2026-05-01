#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include <libserial/SerialPort.h>


namespace mobile_base_hardware
{

class MobileBaseHardWare : public hardware_interface::SystemInterface
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
    std::shared_ptr<LibSerial::SerialPort> serial_port_ ;
    std::string port_;

    double hw_left_position_  = 0.0;
    double hw_left_velocity_  = 0.0;
    double hw_right_position_ = 0.0;
    double hw_right_velocity_ = 0.0;

    double cmd_left_velocity_  = 0.0;
    double cmd_right_velocity_ = 0.0;


}; //class

} //namespace mobile base hardware 



#endif