#include "hw_interface/led_hardware_interface.hpp"

namespace led_blink_test
{

hardware_interface::CallbackReturn LedBlinkTest::on_init
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


std::vector<hardware_interface::StateInterface> LedBlinkTest::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> states;


    //right wheel position and velocity
    states.emplace_back(
        hardware_interface::StateInterface(
            "base_esp_joint","state",&read_state));



    return states;

}

std::vector<hardware_interface::CommandInterface> LedBlinkTest::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> commands;

    commands.emplace_back(
        hardware_interface::CommandInterface(
            "base_esp_joint","state",&write_state));


    return commands;

}

hardware_interface::CallbackReturn LedBlinkTest::on_configure
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

hardware_interface::CallbackReturn LedBlinkTest::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{

    (void)previous_state;
    read_state = 0.0;

    serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);


    serial_port_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_->SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);

    return hardware_interface::CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn LedBlinkTest::on_deactivate
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

hardware_interface::return_type LedBlinkTest::read
    (const rclcpp::Time &time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    std::string responce;

    if (serial_port_->IsDataAvailable()== true)
    {
        

        try
        {
            serial_port_->ReadLine(responce,'\n',100);
            
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
    }

    

    return hardware_interface::return_type::OK;

}

hardware_interface::return_type LedBlinkTest::write
    (const rclcpp::Time &time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    std::string command_ = "";
    if (write_state > 0.5) {
        command_ = "LED:1.0\n";
    } else {
        command_ = "LED:0.0\n";
    }

    serial_port_->Write(command_);

    serial_port_->DrainWriteBuffer();

    return hardware_interface::return_type::OK;

}

} //namespce mobile_base_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(led_blink_test::LedBlinkTest,hardware_interface::SystemInterface)