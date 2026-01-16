import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import DigitalInputDevice

class VoltageStatusNode(Node):
    def __init__(self):
        super().__init__('voltage_status_node')
        
        # Initialize GPIO 21 as a digital input
        # Note: The level shifter has its own 10k pull-up resistors,
        # so we don't necessarily need the Pi's internal pull-up.
        self.voltage_pin = DigitalInputDevice(16)
        
        self.publisher_ = self.create_publisher(Bool, 'is_voltage_present', 10)
        
        # Check every 0.1 seconds (10Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Binary Voltage Detector (via Level Shifter) is running...")

    def timer_callback(self):
        msg = Bool()
        # msg.data will be True if HV1 is 24V, and False if HV1 is 0V
        msg.data = self.voltage_pin.is_active
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoltageStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
