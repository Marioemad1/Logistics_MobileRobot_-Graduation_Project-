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

    address_ = 0x68;
    bus_= "/dev/i2c-1";

    driver_ = std::make_shared<MPU6050Driver>(bus_,address_);
    return hardware_interface::CallbackReturn::SUCCESS;
    
}


std::vector<hardware_interface::StateInterface> Mpu6050HardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> states;

    std::string sensor_name_link = "imu_link"; //LINK URDF


    //linear acceleration
    states.emplace_back(
        hardware_interface::StateInterface(
            sensor_name_link,"linear_acceleration.x",&linear_acceleration_[0]));
    states.emplace_back(
            hardware_interface::StateInterface(
                sensor_name_link,"linear_acceleration.y",&linear_acceleration_[1]));
    states.emplace_back(
            hardware_interface::StateInterface(
                sensor_name_link,"linear_acceleration.z",&linear_acceleration_[2]));

    //anguler acceleration
    states.emplace_back(
        hardware_interface::StateInterface(
            sensor_name_link,"angular_velocity.x",&angular_velocity_[0]));
    states.emplace_back(
            hardware_interface::StateInterface(
                sensor_name_link,"angular_velocity.y",&angular_velocity_[1]));
    states.emplace_back(
            hardware_interface::StateInterface(
                sensor_name_link,"angular_velocity.z",&angular_velocity_[2]));


    states.emplace_back(
        hardware_interface::StateInterface(
            sensor_name_link, "orientation.x", &orientation_[0]));
    states.emplace_back(
        hardware_interface::StateInterface(
            sensor_name_link, "orientation.y", &orientation_[1]));
    states.emplace_back(
        hardware_interface::StateInterface(
            sensor_name_link, "orientation.z", &orientation_[2]));
    states.emplace_back(
        hardware_interface::StateInterface(
            sensor_name_link, "orientation.w", &orientation_[3]));

    return states;

}

std::vector<hardware_interface::CommandInterface> Mpu6050HardwareInterface::export_command_interfaces()
{
    return{}; //no write here 

}

hardware_interface::CallbackReturn Mpu6050HardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    if (!driver_->init())
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

    MPU6050DATA mpu_comming_data = driver_->readData();

    //update the accleration in linear
    linear_acceleration_[0] = mpu_comming_data.accel_x;
    linear_acceleration_[1] = mpu_comming_data.accel_y;
    linear_acceleration_[2] = mpu_comming_data.accel_z;


    //update the anguler velcoity
    angular_velocity_[0]= mpu_comming_data.gyro_x ;
    angular_velocity_[1]= mpu_comming_data.gyro_y ;
    angular_velocity_[2]= mpu_comming_data.gyro_z ;
    
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
