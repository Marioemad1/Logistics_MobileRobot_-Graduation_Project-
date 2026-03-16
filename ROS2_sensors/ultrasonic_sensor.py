#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from gpiozero import DigitalOutputDevice
from gpiozero import DigitalInputDevice
from sensor_msgs.msg import Range


class UltraSonicSensor(Node): #create the class and inharet from node
    def __init__(self): 
        super().__init__("ultra_sonic_sensor") #call the init from the Node class 
       #parameter configuration
        self.declare_parameter("trig_pin", 17 ) #this is BCM -> GPIO17 Physical 11
        self.declare_parameter("echo_pin", 13 )#this is BCM-> GPIO13 Physical 33
        self.declare_parameter("hz_timer", 0.1)
        self.declare_parameter("time_out",0.03)
        self.declare_parameter("frame_id","ultrasonic_link")
         #pin configuration
        self.trig_pin_ = self.get_parameter("trig_pin").value
        self.echo_pin_ = self.get_parameter("echo_pin").value

        self.time_out_ = self.get_parameter("time_out").value

        self.timer_hz_ = self.get_parameter("hz_timer").value

        self.fram_id_ = self.get_parameter("frame_id").value

        self.trig_= DigitalOutputDevice(pin = self.trig_pin_,initial_value = False)
        self.echo_= DigitalInputDevice(pin= self.echo_pin_, pull_up= False)
        #small dely for settings
        time.sleep(0.05)
        
        #publisher
        self.ultrasonic_readings_pub_=self.create_publisher(Range,"ultrasonic_readings",10)

        #timer to active the sensor
        self.active_sensor_ = self.create_timer(self.timer_hz_,self.callback_sensor)

        self.get_logger().info("the_Ultrasonic_sensor_is_running....")

    def callback_sensor(self):
        #sending High pulse for 0.00001 10 microsecnds
        self.trig_.on()
        time.sleep(0.00001)
        self.trig_.off()

        time_out_ = self.time_out_ #time out to not block 
        start_time_ = time.time()

        while self.echo_.value == 0:
            if time.time()- start_time_ > time_out_:
                self.get_logger().warn(f"the echo is low for more than the timeout:{time_out_}")
                return
        #after going out from this loop this means that the echo is resived so i recoreded the start of this pulse        
        pulse_start = time.time()

        start_time_= time.time()
        while self.echo_.value == 1:
            if time.time() - start_time_ > time_out_:
                self.get_logger().warn(f"the echo is high for more than the timeout:{time_out_}")
                return
        
        pulse_end_ = time.time()

        distance_ = (pulse_end_-pulse_start)*17150 / 100.0 #the equation to convert the time to distance in meters 

        msg = Range()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.fram_id_ # NOTE:this is the frame name of the msg 
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.5 #this means 30 degree in radian 
        msg.min_range = 0.02
        msg.max_range = 4
        msg.range = distance_

        self.ultrasonic_readings_pub_.publish(msg)
        self.get_logger().info(f"distance:{distance_:.2f}m")
        





def main(args=None):
    rclpy.init(args=args)   #init the communication
    node=UltraSonicSensor()   #create a object from the class
    rclpy.spin(node)        #make the node spin
    rclpy.shutdown()        #shutdow the node



#if the name main called in the terminal call the main function
if __name__ == "__main__":
    main()
