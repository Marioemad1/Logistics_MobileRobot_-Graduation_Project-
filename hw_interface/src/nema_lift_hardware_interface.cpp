#include "hw_interface/nema_lift_hardware_interface.hpp"
#include "hw_interface/shared_serial_port.hpp"

namespace nema_lift_hardware_interface
{

hardware_interface::CallbackReturn NemaLiftHardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info)
{
    if(hardware_interface::SystemInterface::on_init(info) !=
       hardware_interface::CallbackReturn::SUCCESS )
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    read_state = 0.0;
    write_state= 0.0;

    port_ = "/dev/ttyUSB0";

    return hardware_interface::CallbackReturn::SUCCESS;
    
}


std::vector<hardware_interface::StateInterface> NemaLiftHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> states;


    //right wheel position and velocity
    states.emplace_back(
        hardware_interface::StateInterface(
            "base_esp_joint","state",&read_state));



    return states;

}

std::vector<hardware_interface::CommandInterface> NemaLiftHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> commands;

    commands.emplace_back(
        hardware_interface::CommandInterface(
            "base_esp_joint","state",&write_state));


    return commands;

}

hardware_interface::CallbackReturn NemaLiftHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    auto& shared_serial = hw_interface::SharedSerialPort::getInstance();

    if (shared_serial.port->IsOpen() != true )
    {
        shared_serial.port->Open(port_);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NemaLiftHardwareInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{

    (void)previous_state;
    read_state = 0.0;
    auto& shared_serial = hw_interface::SharedSerialPort::getInstance();

    shared_serial.port->SetBaudRate(LibSerial::BaudRate::BAUD_115200);


    shared_serial.port->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    shared_serial.port->SetParity(LibSerial::Parity::PARITY_NONE);
    shared_serial.port->SetStopBits(LibSerial::StopBits::STOP_BITS_1);

    return hardware_interface::CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn NemaLiftHardwareInterface::on_deactivate
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

hardware_interface::return_type NemaLiftHardwareInterface::read
    (const rclcpp::Time &time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    auto& shared_serial = hw_interface::SharedSerialPort::getInstance();
    std::lock_guard<std::mutex> lock(shared_serial.serial_mutex);

    std::string responce;

    
    if (shared_serial.port->IsDataAvailable()== true)
    {
        try
        {
            shared_serial.port->ReadLine(responce,'\n',100);
            
            if (!responce.empty() && responce.back() == '\r') {
                responce.pop_back();
            }

            // Safety Step 2: Try to do the math
            if (!responce.empty()) {
                read_state = std::stod(responce);
            }
        }
        catch(const LibSerial::ReadTimeout& e)
        {
           /*  return hardware_interface::return_type::ERROR; */
        }
        catch(const std::exception& e) // <--- JUST ADD THIS CATCH
        {
            // This safely swallows the "ACK" crash from stod!
        }
    }

    

    return hardware_interface::return_type::OK;

}

hardware_interface::return_type NemaLiftHardwareInterface::write
    (const rclcpp::Time &time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    auto& shared_serial = hw_interface::SharedSerialPort::getInstance();

    std::string command_ = "";
    if (write_state > 0.5) {
        command_ = "N:1.0\n";
    } 
    else if(write_state < -0.5)
    {
        command_ = "N:-1.0\n";
    }
    else
    {
        command_ = "N:0.0\n";
    }

    std::lock_guard<std::mutex> lock(shared_serial.serial_mutex);

    shared_serial.port->Write(command_);

    shared_serial.port->DrainWriteBuffer();

    return hardware_interface::return_type::OK;

}

} //namespce mobile_base_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nema_lift_hardware_interface::NemaLiftHardwareInterface,hardware_interface::SystemInterface)