#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import time

#don't forget here to add the srv sev types when time comes 

class IOT_Connection(Node): #create the class and inharet from node
    def __init__(self): 
        super().__init__("Iot_Bridge") #call the init from the Node class 
        self.declare_parameter("MQTT_BROKER","local_host") #this is where you but your cloud address
        self.declare_parameter("MQTT_PORT",1883)
        self.declare_parameter("MQTT_TOPIC","robot/command") #the topic name that the cloud will send topics on

        self.MQT_Broker_=self.get_parameter("MQTT_BROKER").value
        self.MQT_Port_=self.get_parameter("MQTT_PORT").value
        self.MQT_Topic_=self.get_parameter("MQTT_TOPIC").value

        #add here the list of clints for the servese you need 


        self.cloud_connected_= False
        self.command_last_time_= 0
        self.MQTT_clinet_=mqtt.Client(client_id="ros2_iot_bridge",clean_session=True)
        self.MQTT_clinet_.on_connect= self.mqtt_on_connect
        self.MQTT_clinet_.on_disconnect= self.mqtt_on_disconnect
        self.MQTT_clinet_.on_message= self.mqtt_on_massage

        self.MQTT_command_ = None

        self.MQTT_clinet_.will_set(topic= "robot/status", payload="OFFLINE" , qos= 1 , retain= True)
        self.MQTT_clinet_.reconnect_delay_set(1,30)

        self.MQTT_clinet_.connect(self.MQT_Broker_,self.MQT_Port_,60)

        self.MQTT_clinet_.loop_start()



    def mqtt_on_connect(self,client, userdata, flags, rc):
        if rc == 0 :
            self.cloud_connected_ = True
            self.get_logger().info("[MQTT]the cloud is connected to the node.....")
            self.MQTT_clinet_.subscribe(self.MQT_Topic_,1)

            self.MQTT_clinet_.publish("robot/status","ONLINE", 1 , True)

        else:
            self.get_logger().warn(f"[MQTT] faild to conncet with code of {rc}")

    def mqtt_on_disconnect(self,client, userdata, rc):

        self.cloud_connected_= False
        self.get_logger().warn("[MQTT] Disconnectd from IOT cloud...")

        if rc != 0:
            self.get_logger().warn("[MQTT] Unexpected disconnection, retrying...")

    def mqtt_on_massage(self,client, userdata, msg):
        self.command_last_time_ = time.time()

        self.MQTT_command_ = msg.payload.decode()

        self.get_logger().info(f"[MQTT] the command recived {self.MQTT_command_}")


    #and add the sev call back for responce future and requests 
        





def main(args=None):
    rclpy.init(args=args)   #init the communication
    node=IOT_Connection()   #create a object from the class
    rclpy.spin(node)        #make the node spin
    rclpy.shutdown()        #shutdow the node



#if the name main called in the terminal call the main function
if __name__ == "__main__":
    main()