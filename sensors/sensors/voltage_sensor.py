#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gpiozero import DigitalInputDevice
from std_srvs.srv import SetBool

class VoltageSensor(Node): #create the class and inharet from node
    def __init__(self): 
        super().__init__("voltage_sensor") #call the init from the Node class
        self.declare_parameter("voltage_in_pin",18)
        self.voltage_pin_ = self.get_parameter("voltage_in_pin").value
        self.in_voltage = DigitalInputDevice(self.voltage_pin_,pull_up= False)
        self.servic_the_state_ = self.create_service(SetBool,"switch_state",self.call_switch_state_srv) 

    def call_switch_state_srv(self, request: SetBool.Request , response: SetBool.Response):
        if request.data == 1:
            response.success = bool(self.in_voltage.value)
            response.message = f"the request is write the state: {self.in_voltage.value}"
        else:
            response.success = bool(self.in_voltage.value)
            response.message = f"the request is wrong but the state is {self.in_voltage.value}"
        
        return response

def main(args=None):
    rclpy.init(args=args)   #init the communication
    node=VoltageSensor()   #create a object from the class
    rclpy.spin(node)        #make the node spin
    rclpy.shutdown()        #shutdow the node



#if the name main called in the terminal call the main function
if __name__ == "__main__":
    main()