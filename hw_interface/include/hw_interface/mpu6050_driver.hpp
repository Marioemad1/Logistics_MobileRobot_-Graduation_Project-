#ifndef MPU6050_DRIVER_HPP
#define MPU6050_DRIVER_HPP


#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <cmath>

// Standard MPU6050 Registers
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

#define GRAVITY 9.80665
#define ACCEL_SCALE 16384.0
#define GYRO_SCALE 131.0

struct MPU6050DATA
{
    double accel_x; 
    double accel_y; 
    double accel_z;
    //------------ 
    double gyro_x;  
    double gyro_y; 
    double gyro_z;  
};


class MPU6050Driver {
    public:
        MPU6050Driver(const char*bus ="/dev/i2c-1",int address = 0x68){

            this->bus_filename_  = bus;
            this->sensor_address_= address;
            this->file_i2c_ = -1; //set it to unopen condetion 
        }

        ~MPU6050Driver() {
        if (this->file_i2c_ >= 0) {
            close(this->file_i2c_);
            }
        }


        bool init() //init the mpu the wake up and configure
        {
            //open the i2c bus
            this->file_i2c_ = open(this->bus_filename_,O_RDWR);
            if(this->file_i2c_< 0)
            {
                std::cerr<<" [MPU6050] Faild to open the i2c file"<<std::endl;
                return false;
            }

            //connect to the sensor address
            if (ioctl(this->file_i2c_,I2C_SLAVE,this->sensor_address_) < 0 )
            {
                std::cerr<<"[MPU6050] Faild to connect to the sensor address"<<std::endl;
                return false;
            }

            //wake up the sensor
            uint8_t buffer[2] = {PWR_MGMT_1,0x00}; //the power address and the msg of wkup
            if (write(this->file_i2c_,buffer,2) != 2)
            {
                std::cerr<<"[MPU6050] Faild to wake up the sensor"<<std::endl;
                return false;
            }

            std::cout<<"[MPU6050] Initialized successfully"<<std::endl;
            return true;
        }

        MPU6050DATA readData()
        {
            MPU6050DATA data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            uint8_t reg[1] = {ACCEL_XOUT_H}; //THE START ADDRESS
            uint8_t buffer[14] = {0}; //where we will recive the data

            if (write(this->file_i2c_,reg,1) !=1 )
            {
                std::cerr<<"[MPU6050] Faild to set read register"<<std::endl;
                return data ;
            }

            if (read(this->file_i2c_,buffer,14) != 14 )
            {
                std::cerr<<"[MPU6050] Faild to read from the register"<<std::endl;
                return data ;
            }

            // Combine High and Low bytes (16-bit signed integers)
            int16_t raw_accel_x = (buffer[0] << 8) | buffer[1];
            int16_t raw_accel_y = (buffer[2] << 8) | buffer[3];
            int16_t raw_accel_z = (buffer[4] << 8) | buffer[5];
            // bytes 6 and 7 are temperature, skipping for now
            int16_t raw_gyro_x  = (buffer[8] << 8) | buffer[9];
            int16_t raw_gyro_y  = (buffer[10] << 8) | buffer[11];
            int16_t raw_gyro_z  = (buffer[12] << 8) | buffer[13];

            // Convert raw values to ROS 2 standard units
            data.accel_x = (raw_accel_x / ACCEL_SCALE) * GRAVITY;
            data.accel_y = (raw_accel_y / ACCEL_SCALE) * GRAVITY;
            data.accel_z = (raw_accel_z / ACCEL_SCALE) * GRAVITY;

            data.gyro_x = (raw_gyro_x / GYRO_SCALE) * (M_PI / 180.0);
            data.gyro_y = (raw_gyro_y / GYRO_SCALE) * (M_PI / 180.0);
            data.gyro_z = (raw_gyro_z / GYRO_SCALE) * (M_PI / 180.0);

            return data;
        }
    
    
    private:
        int file_i2c_;
        int sensor_address_;
        const char* bus_filename_;
};

#endif