#include "rclcpp/rclcpp.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <algorithm>

#define POINTER_CONVERSION  0x00
#define POINTER_CONFIG  0x01

#define VOLTAGE_DIVIDER_RATIO 5.0
#define BATTERY_MAX_V  25.0
#define BATTERY_MIN_V 20.0

using namespace std::chrono_literals;

struct BatteryData {
    double voltage;    // Actual battery voltage
    double percentage; // 0.0 to 100.0 %
};

class BattryVoltageNode : public rclcpp::Node 
{
    public: //puplic
        BattryVoltageNode() : Node("battery_voltage") 
        {
            this->declare_parameter("file_name","/dev/i2c-1");
            this->declare_parameter<int>("sensor_add",0x48);
            sensor_address_ = this->get_parameter("sensor_add").as_int();
            file_i2c_ = -1; //unset
            bus_file_name = this->get_parameter("file_name").as_string();
            timer_ = this->create_wall_timer(10s,std::bind(&BattryVoltageNode::timer_callback,this));


            RCLCPP_INFO(this->get_logger(),"The Battery node is running...");
        }
        
        
    private:

    bool init()
    {
        this->file_i2c_ = open(this->bus_file_name.c_str(),O_RDWR);
        if (this->file_i2c_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(),"[ADC]FAILD TO OPEN I2C FILE ");
        }

        if (ioctl(this->file_i2c_,I2C_SLAVE,sensor_address_) < 0)
        {
            RCLCPP_ERROR(this->get_logger(),"[ADC] FAILD TO CONNECT TO THE SENSOR");
        }

        RCLCPP_INFO(this->get_logger(),"[ADC] Initialized successfully!");
        return true;
    }

    BatteryData ReadBatter()
    {
        BatteryData data = {0.0,0.0};
        
        uint8_t config[3] = {0x01, 0xC0, 0x83};;
        if (write(this->file_i2c_,config,3) != 3)
        {
            RCLCPP_ERROR(this->get_logger(),"[ADC] FAILD TO CONFIGURE THE SENSOR");
            return data;
        }
        usleep(10000);

        uint8_t reg[1]= {POINTER_CONVERSION};
        write(this->file_i2c_,reg,1);

        uint8_t comming_data[2] = {0};
        if (read(this->file_i2c_,comming_data, 2) != 2)
        {
            RCLCPP_ERROR(this->get_logger(),"[ADC] FAILD TO READ THE DATA FROM SENSOR");
            return data;
        }

        int16_t raw_adc = (comming_data[0] << 8) | comming_data[1];
        double adc_voltage = raw_adc * 0.0001875;
        data.voltage = adc_voltage * VOLTAGE_DIVIDER_RATIO;

        double raw_percentage = (
            (data.voltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V)) * 100.0;

        data.percentage = std::clamp(raw_percentage, 0.0, 100.0);
        return data;
    }

    void timer_callback()
    {
        
    }

    int file_i2c_;
    int sensor_address_;
    std::string bus_file_name;
    rclcpp::TimerBase::SharedPtr timer_;

    

    
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<BattryVoltageNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
