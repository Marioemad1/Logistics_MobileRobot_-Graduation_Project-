#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from gpiozero import DigitalOutputDevice

#if the bms node is converted to a server don't forget to make this clinet 

class BatteryLEDS(Node): #create the class and inharet from node
    def __init__(self): 
        super().__init__("battery_leds") #call the init from the Node class 
        self.declare_parameter("led1",13)
        self.declare_parameter("led2",19)
        self.declare_parameter("led3",26)
        self.LED_1_= self.get_parameter("led1").value
        self.LED_2_= self.get_parameter("led2").value
        self.LED_3_= self.get_parameter("led3").value

        #don't forget to change this with the right command
        self.persent_command_ =0000000
        self.led_1_ = DigitalOutputDevice(self.LED_1_, initial_value= False)
        self.led_2_ = DigitalOutputDevice(self.LED_2_, initial_value= False)
        self.led_3_ = DigitalOutputDevice(self.LED_3_, initial_value= False)

        self.BMS_sub_= self.create_subscription(UInt8MultiArray,"BMS_data",self.callback_bms_data,10)
        self.get_percent_= self.create_publisher(UInt8MultiArray,"BMS_command",10)
        self.timer_publisher_=self.create_timer(1.0,self.call_the_bms)

        self.get_logger().info("the_battery_leds_is_running...")



    def call_the_bms(self):
        #don't forget to add acheck here that this data is the persent data not any thing else
        msg = UInt8MultiArray()
        msg.data =  list(self.persent_command_)
        self.get_percent_.publish(msg)

    def callback_bms_data(self, data : UInt8MultiArray ):
        charge_persent_ = data.data

        if charge_persent_ > 66:
            self.led_1_.on()
            self.led_2_.on()
            self.led_3_.on()
        elif 66 > charge_persent_ >33 :
            self.led_1_.on()
            self.led_2_.on()
            self.led_3_.off()
        elif charge_persent_ < 33 :
            self.led_1_.on()
            self.led_2_.off()
            self.led_3_.off()
        else:
            self.led_1_.off()
            self.led_2_.off()
            self.led_3_.off()


def main(args=None):
    rclpy.init(args=args)   #init the communication
    node=BatteryLEDS()   #create a object from the class
    rclpy.spin(node)        #make the node spin
    rclpy.shutdown()        #shutdow the node



#if the name main called in the terminal call the main function
if __name__ == "__main__":
    main()