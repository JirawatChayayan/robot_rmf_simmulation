#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
import sys
from rcl_interfaces.msg import ParameterDescriptor
import argparse

class InitPoseNode(Node):
    def __init__(self,x,y,z,w):
        super().__init__('init_pose_node')



        # # Declare parameters
        # self.declare_parameter('x', '5.19')
        # self.declare_parameter('y', '-0.68')
        # self.declare_parameter('z', '-0.9317268443466659')
        # self.declare_parameter('w', '0.36315986496831365')


        # # Get parameters
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, f'initialpose', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.publish_count = 0
        self.max_publish_count = 5
        self.test = {}


    def dataRun(self):
        data = {
            'x': self.x,
            'y': self.y,
            'rot': {
                'z': self.z,
                'w': self.w
            }
        }
        self.test = data
        initial_pose = PoseWithCovarianceStamped()
        # Set the initial pose (x, y, z, orientation)
        initial_pose.pose.pose.position.x = float(data['x'])
        initial_pose.pose.pose.position.y = float(data['y'])
        initial_pose.pose.pose.position.z = 0.0  # You can set z if needed
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = float(data['rot']['z'])
        initial_pose.pose.pose.orientation.w = float(data['rot']['w'])

        # Set the covariance to identity (no uncertainty)
        # initial_pose.pose.covariance = [0.0] * 36  # 6x6 covariance matrix
        # initial_pose.pose.covariance[0] = 0.25
        # initial_pose.pose.covariance[7] = 0.25
        # initial_pose.pose.covariance[35] = 0.06853891909122467


        # Initialize header
        initial_pose.header = Header()
        initial_pose.header.frame_id = 'map'  # Change as needed
        initial_pose.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
        return initial_pose


    def timer_callback(self):
        if self.publish_count < self.max_publish_count:
            data = self.dataRun()
            self.publisher_.publish(data)
            self.get_logger().info(f'topic {self.get_namespace()}/initialpose has Publishing initial pose: {self.test}')
            self.publish_count += 1
        else:
            self.get_logger().info('Max publish count reached. Shutting down...')
            self.timer.cancel()  # Stop the timer
            raise Exception('Finished')

def main(args=None):
    rclpy.init(args=args)


    # Remove ROS-specific arguments and any unwanted path arguments
    args_without_ros = rclpy.utilities.remove_ros_args(args)

    # Handle potential issues with extraneous arguments
    args_without_ros = [arg for arg in args_without_ros if not arg.endswith('.py')]

    # Parse the custom arguments (x, y, z, w)
    parser = argparse.ArgumentParser(description='Simple ROS 2 node with arguments.')
    parser.add_argument('-x', type=float, required=True, help='X coordinate value')
    parser.add_argument('-y', type=float, required=True, help='Y coordinate value')
    parser.add_argument('-z', type=float, required=True, help='Z coordinate value')
    parser.add_argument('-w', type=float, required=True, help='W value (orientation)')

    parsed_args = parser.parse_args(args_without_ros)

    try:

        init_pose_node = InitPoseNode(parsed_args.x,parsed_args.y,parsed_args.z,parsed_args.w)
        rclpy.spin(init_pose_node)
    except:
        init_pose_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
