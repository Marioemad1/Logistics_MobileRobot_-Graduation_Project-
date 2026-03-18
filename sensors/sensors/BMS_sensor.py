#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import BatteryState
import smbus2

class ADCReadings(Node): #create the class and inharet from node
    def __init__(self): 
        super().__init__("Battery_char")
        self.declare_parameter("bus_number",1)
        self.declare_parameter("timer_hz",0.5)
        self.declare_parameter("VOLTAGE_DIVIDER",11.0)
        self.declare_parameter("V_MAX",16.8)
        self.declare_parameter("V_MIN",13.2)
        self.declare_parameter("BATTERY_CAPACITY_AH",10.0)
        self.declare_parameter("CURRENT_ZERO",2.5)
        self.declare_parameter("CURRENT_SENS",0.1)
        
        self.VOLTAGE_DIVIDER_ = self.get_parameter("VOLTAGE_DIVIDER").value

        self.V_MAX_ = self.get_parameter("V_MAX").value
        self.V_MIN_ = self.get_parameter("V_MIN").value

        self.BATTERY_CAPACITY_AH_ = self.get_parameter("BATTERY_CAPACITY_AH").value  # change to your battery

        self.CURRENT_ZERO_ = self.get_parameter("CURRENT_ZERO").value
        self.CURRENT_SENS_ = self.get_parameter("CURRENT_SENS").value

        self.Bus_ = smbus2.SMBus(self.get_parameter("bus_number").value)
        self.battery_timer_ = self.create_timer(self.get_parameter("timer_hz").value,self.call_update_battery_state)

        self.publish_battery_= self.create_publisher(BatteryState,"battery_state",10)

        self.get_logger().info("the battery state is running....")


    def call_update_battery_state(self):
        
        current_readings = self.ADC_read_(1)
        voltag_readings  = self.ADC_read_(0)


        battery_voltage = voltag_readings * self.VOLTAGE_DIVIDER_
        battery_current = (current_readings - self.CURRENT_ZERO_) / self.CURRENT_SENS_

        soc = (battery_voltage - self.V_MIN_) / (self.V_MAX_ - self.V_MIN_)
        soc = max(0.0, min(1.0, soc))

                # Battery state message
        msg = BatteryState()
        msg.voltage = battery_voltage
        msg.current = battery_current
        msg.percentage = soc

        msg.capacity = self.BATTERY_CAPACITY_AH_
        msg.charge = soc * self.BATTERY_CAPACITY_AH_

        msg.present = True

        if battery_current > 0.2:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif battery_current < -0.2:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

        self.publish_battery_.publish(msg)

        self.get_logger().info(
            f"V={battery_voltage:.2f}V | I={battery_current:.2f}A | SOC={soc*100:.1f}%")




    def ADC_read_(self, channal_number ):

        mux_map = {
        0: 0x4000,  # A0
        1: 0x5000,  # A1
        2: 0x6000,  # A2
        3: 0x7000,  # A3
        }

        config = (
        0x8000 |            # Start conversion
        mux_map[channal_number] |  # Channel select
        0x0200 |            # ±4.096V
        0x0100 |            # Single-shot
        0x0080 |            # 128 SPS
        0x0003              # Disable comparator
        )

        self.Bus_.write_i2c_block_data(
        0x48, 0x01,
        [(config >> 8) & 0xFF, config & 0xFF]
        )

        time.sleep(0.01)

        data = self.Bus_.read_i2c_block_data(0x48, 0x00, 2)
        raw = (data[0] << 8) | data[1]

        if raw > 0x7FFF:
            raw -= 0x10000

        result = raw * 4.096 / 32768.0

        return result






def main(args=None):
    rclpy.init(args=args)   #init the communication
    node=ADCReadings()      #create a object from the class
    rclpy.spin(node)        #make the node spin
    rclpy.shutdown()        #shutdow the node



#if the name main called in the terminal call the main function
if __name__ == "__main__":
    main()