import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState

class FakeBattery(Node):
    def __init__(self):
        super().__init__("fake_battery_node")
        self.increment = 0.3
        self.voltage = 12.6
        self.batt_bub = self.create_publisher(BatteryState,"battery_state",10)
        self.timer1 = self.create_timer(1.0,self.timer1_callback)
    
    def timer1_callback(self):
        msg = BatteryState()
        msg.header.frame_id = "battery_frame"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = 12.8
        msg.percentage = self.increment
        msg.current = -1.5
        msg.charge = 50.0
        msg.capacity = 100.0
        msg.design_capacity = 100.0
        msg.power_supply_status = 2
        msg.power_supply_health = 1
        msg.power_supply_technology = 2
        msg.present = True
        if(self.increment > 1.0):
            self.increment = 0.3
        self.increment += 0.1
        self.batt_bub.publish(msg)

def main():
    rclpy.init()
    fb = FakeBattery()
    rclpy.spin(fb)
    fb.destroy_node()
    rclpy.shutdown()