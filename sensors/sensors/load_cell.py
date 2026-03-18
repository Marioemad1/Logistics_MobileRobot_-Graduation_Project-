#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hx711_multi import hx711
from std_msgs.msg import Float64
try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

class LoadCell(Node): #create the class and inharet from node
    def __init__(self): 
        super().__init__("load_cell") #call the init from the Node class 
        self.declare_parameter("DT_PIN",5 )   #this IN BCM->GPIO5 iN Pyhsical-> 29
        self.declare_parameter("SCK_PIN",6)   #this IN BCM->GPIO6 IN Pyhsical-> 31
        self.declare_parameter("timer_hz",0.5)

        self.dt_pin_  = self.get_parameter("DT_PIN").value
        self.sck_pin_ = self.get_parameter("SCK_PIN").value

        self.timer_hz_=self.get_parameter("timer_hz").value

        self.list_dt_pins_ = [self.dt_pin_]
        self.hx_= hx711(self.list_dt_pins_,self.sck_pin_, byte_format="MSB", gain=128)

        self.load_cell_pub_ = self.create_publisher(Float64,"the_weight",10)
        self.timer_=self.create_timer(0.5,self.callback_load_cell)

        self.get_logger().info("Initializing...")
        self.hx_.reset()
        self.get_logger().info("Taring (Zeroing)... remove weight.")
        self.hx_.zero() # Sets the current reading to 0
        self.get_logger().info("Ready to weigh.")
        self.get_logger().info("the_load_sensor_is_running...")

    def callback_load_cell(self):

        
        msg = Float64()
        msg.data = self.hx_.get_weight()[0]

        self.load_cell_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)   #init the communication
    node=LoadCell()   #create a object from the class
    rclpy.spin(node)        #make the node spin
    if GPIO is not None:
        GPIO.cleanup()
    rclpy.shutdown()        #shutdow the node
    rclpy.spin(node)        #make the node spin
    GPIO.cleanup()
    rclpy.shutdown()        #shutdow the node



#if the name main called in the terminal call the main function
if __name__ == "__main__":
    main()