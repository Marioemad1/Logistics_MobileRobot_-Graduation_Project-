#include "hw_interface/mobile_base_hardware_interface.hpp"
#include "hw_interface/shared_serial_port.hpp"

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
    port_ = "/dev/ttyACM0";

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
    auto& shared_serial = hw_interface::SharedSerialPort::getInstance();
    if (shared_serial.port->IsOpen() != true )
    {
        shared_serial.port->Open(port_);

        shared_serial.port->SetDTR(true);
        shared_serial.port->SetRTS(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        shared_serial.port->SetDTR(false);
        shared_serial.port->SetRTS(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardWare::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{

    (void)previous_state;
    auto& shared_serial = hw_interface::SharedSerialPort::getInstance();
    shared_serial.port->SetBaudRate(LibSerial::BaudRate::BAUD_115200);


    shared_serial.port->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    shared_serial.port->SetParity(LibSerial::Parity::PARITY_NONE);
    shared_serial.port->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    return hardware_interface::CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn MobileBaseHardWare::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    auto& shared_serial = hw_interface::SharedSerialPort::getInstance();
    shared_serial.port->Close();
    if (shared_serial.port->IsOpen() == true )
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

    double dt = period.seconds();

    
    hw_right_velocity_ = cmd_right_velocity_;
    hw_left_velocity_  = cmd_left_velocity_;

    
    hw_right_position_ += (hw_right_velocity_ * dt);
    hw_left_position_  += (hw_left_velocity_ * dt);

    return hardware_interface::return_type::OK;

}

hardware_interface::return_type MobileBaseHardWare::write
    (const rclcpp::Time &time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    auto& shared_serial = hw_interface::SharedSerialPort::getInstance();

    std::lock_guard<std::mutex> lock(shared_serial.serial_mutex);
    
    std::string right_wheel_vel = std::to_string(cmd_right_velocity_);
    std::string left_wheel_vel = std::to_string(cmd_left_velocity_);

    std::string command_ = "R:"+ right_wheel_vel + "," + "L:" + left_wheel_vel+ "\n" ;  

    
    shared_serial.port->Write(command_);

    shared_serial.port->DrainWriteBuffer();

    return hardware_interface::return_type::OK;

}

} //namespce mobile_base_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardWare,hardware_interface::SystemInterface)
