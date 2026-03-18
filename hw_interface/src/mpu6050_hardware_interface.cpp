#include "hw_interface/mpu_hardware_interface.hpp"

namespace mpu6050_hardware
{

hardware_interface::CallbackReturn Mpu6050HardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info)
{
    if(hardware_interface::SystemInterface::on_init(info) !=
       hardware_interface::CallbackReturn::SUCCESS )
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
    
}


std::vector<hardware_interface::StateInterface> Mpu6050HardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> states;


    //right wheel position and velocity
   /*  states.emplace_back(
        hardware_interface::StateInterface(
            "base_right_wheel_joint","position",&hw_right_position_)); */

    return states;

}

std::vector<hardware_interface::CommandInterface> Mpu6050HardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> commands;

    /* commands.emplace_back(
        hardware_interface::CommandInterface(
            "base_right_wheel_joint","velocity",&cmd_right_velocity_)); */

    return commands;

}

hardware_interface::CallbackReturn Mpu6050HardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    if (driver_->init() == 0)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Mpu6050HardwareInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{

    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn Mpu6050HardwareInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Mpu6050HardwareInterface::read
    (const rclcpp::Time &time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;


    return hardware_interface::return_type::OK;

}

hardware_interface::return_type Mpu6050HardwareInterface::write
    (const rclcpp::Time &time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    return hardware_interface::return_type::OK;

}

} //namespce mpu6050_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mpu6050_hardware::Mpu6050HardwareInterface,hardware_interface::SystemInterface)