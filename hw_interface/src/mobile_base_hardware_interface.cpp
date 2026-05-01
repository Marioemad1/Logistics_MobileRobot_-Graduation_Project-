#include "hw_interface/mobile_base_hardware_interface.hpp"

namespace mobile_base_hardware
{

hardware_interface::CallbackReturn MobileBaseHardWare::on_init
    (const hardware_interface::HardwareInfo & info)
{
    if(hardware_interface::SystemInterface::on_init(info) !=
       hardware_interface::CallbackReturn::SUCCESS )
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;
    port_ = "/dev/ttyUSB0";

    cmd_left_velocity_  = 0.0;
    cmd_right_velocity_ = 0.0;

    
    return hardware_interface::CallbackReturn::SUCCESS;
    
}


std::vector<hardware_interface::StateInterface> MobileBaseHardWare::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> states;

    states.emplace_back(hardware_interface::StateInterface("base_right_wheel_joint",
         "position", &hw_right_position_));
    states.emplace_back(hardware_interface::StateInterface("base_right_wheel_joint",
         "velocity", &hw_right_velocity_));
    states.emplace_back(hardware_interface::StateInterface("base_left_wheel_joint",
         "position", &hw_left_position_));
    states.emplace_back(hardware_interface::StateInterface("base_left_wheel_joint",
         "velocity", &hw_left_velocity_));


    //right wheel position and velocity
    return states;

}

std::vector<hardware_interface::CommandInterface> MobileBaseHardWare::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> commands;

    commands.emplace_back(
        hardware_interface::CommandInterface(
            "base_right_wheel_joint","velocity",&cmd_right_velocity_));

    commands.emplace_back(
        hardware_interface::CommandInterface(
            "base_left_wheel_joint","velocity",&cmd_left_velocity_));

    return commands;

}

hardware_interface::CallbackReturn MobileBaseHardWare::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    serial_port_ = std::make_shared<LibSerial::SerialPort>();

    serial_port_->Open(port_);

    if (serial_port_->IsOpen() != true )
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardWare::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{

    (void)previous_state;
    serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);


    serial_port_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_->SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    return hardware_interface::CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn MobileBaseHardWare::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    serial_port_->Close();
    if (serial_port_->IsOpen() == true )
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MobileBaseHardWare::read
    (const rclcpp::Time &time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    if (serial_port_->IsDataAvailable() == true)
    {
        std::string responce;
        try
        {
            serial_port_->ReadLine(responce, '\n', 100);
            
            // Clean up the string just like we did for the LED
            if (!responce.empty() && responce.back() == '\r') {
                responce.pop_back();
            }

            // We will add the string splitting logic (R:1.5,L:-0.5) here later!
        }
        catch(const LibSerial::ReadTimeout& e)
        {
            // CRITICAL: Catch the timeout silently so the robot doesn't crash!
        }
        catch(const std::exception& e)
        {
            /* RCLCPP_WARN(rclcpp::get_logger("MobileBaseHardWare"), "Ignored bad data from ESP32"); */
        }
    }

    return hardware_interface::return_type::OK;

}

hardware_interface::return_type MobileBaseHardWare::write
    (const rclcpp::Time &time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    std::string right_wheel_vel = std::to_string(cmd_right_velocity_);
    std::string left_wheel_vel = std::to_string(cmd_left_velocity_);

    std::string command_ = "R:"+ right_wheel_vel + "," + "L:" + left_wheel_vel+ "\n" ;  

    serial_port_->Write(command_);

    serial_port_->DrainWriteBuffer();

    return hardware_interface::return_type::OK;

}

} //namespce mobile_base_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardWare,hardware_interface::SystemInterface)