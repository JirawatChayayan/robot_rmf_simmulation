import rclpy
from rclpy.node import Node
from rmf_door_msgs.msg import DoorState, DoorRequest
 
class FakeDoor(Node):
    def __init__(self):
        super().__init__("fake_door_node")
        self.door_status = "Close"
        self.door_state_pub = self.create_publisher(DoorState, "door_states",10)
        self.door_request_sub= self.create_subscription(DoorRequest, "door_requests",self.door_callback,10)

    def door_callback(self, msg):
        door_req = msg.requested_mode.value
        door_name = msg.door_name
        door_msg = DoorState()
        if door_req == 2:
            door_msg.door_name = door_name
            door_msg.current_mode.value = 2
            self.door_state_pub.publish(door_msg)
        print("Door : {} , Request : {}",door_name,door_req)


 
def main():
    rclpy.init()
    fd = FakeDoor()
    rclpy.spin(fd)
    fd.destroy_node()
    rclpy.shutdown()
 
main()