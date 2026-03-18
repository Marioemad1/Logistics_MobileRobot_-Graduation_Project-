#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import smbus2
from sensor_msgs.msg import Imu
import math

class MPUSensor(Node): #create the class and inharet from node
    def __init__(self): 
        super().__init__("mpu_sensor") #call the init from the Node class 
        self.declare_parameter("bus_number",1)
        self.declare_parameter("i2c_address", 0x68)
        self.declare_parameter("timer_hz",0.5)
        #this init the i2c bus 1 in the pi for the mpu
        self.bus_= smbus2.SMBus(self.get_parameter("bus_number").value)
        self.MPU_ADDR_= self.get_parameter("i2c_address").value
        #MPU Wake up 
        self.bus_.write_byte_data(self.MPU_ADDR_,0x6B,0)

        self.timer_hz=self.get_parameter("timer_hz").value
        self.MPU_result_pub_ = self.create_publisher(Imu,"MPU",10)
        self.timer_ = self.create_timer(self.timer_hz,self.MPU_get_values)

    # function to read the registers
    def read_word(self, reg):
        high_reg = self.bus_.read_byte_data(self.MPU_ADDR_,reg)
        low_reg = self.bus_.read_byte_data(self.MPU_ADDR_,reg+1)

        value = (high_reg << 8)+ low_reg
        if value > 0x8000:
            value = -((65535-value)+1)
        return value
    
    def MPU_get_values(self):
        accel_x = self.read_word(0x3B) / 16384.0 # +- 2g
        accel_y = self.read_word(0x3D) / 16384.0 # +- 2g
        accel_z = self.read_word(0x3F) / 16384.0 # +- 2g

        gyro_x = self.read_word(0x43) / 131.0 # +- 250 deg/s
        gyro_y = self.read_word(0x45) / 131.0 # +- 250 deg/s
        gyro_z = self.read_word(0x47) / 131.0 # +- 250 deg/s

        msg = Imu()

        msg.linear_acceleration.x= accel_x * 9.81
        msg.linear_acceleration.y= accel_y * 9.81
        msg.linear_acceleration.z= accel_z * 9.81

        msg.angular_velocity.x= gyro_x *(math.pi/180)
        msg.angular_velocity.y= gyro_y *(math.pi/180)
        msg.angular_velocity.z= gyro_z *(math.pi/180)

        self.MPU_result_pub_.publish(msg)
        

        
def main(args=None):
    rclpy.init(args=args)   #init the communication
    node=MPUSensor()   #create a object from the class
    rclpy.spin(node)        #make the node spin
    rclpy.shutdown()        #shutdow the node



#if the name main called in the terminal call the main function
if __name__ == "__main__":
    main()