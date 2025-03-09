#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SimpleTalker(Node):
    def __init__(self):
        super().__init__('simple_talker')
        self.publisher = self.create_publisher(String, 'robot_message', 10)
        # 确保发布者有时间注册
        time.sleep(1)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        self.get_logger().info('Publisher created. Topic: robot_message')
        
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from Raspberry Pi! Count: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1

def main():
    print('Starting ROS2 node...')
    rclpy.init()
    node = SimpleTalker()
    
    try:
        print('Node initialized, spinning...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Node stopped by keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('Node shutdown complete')

if __name__ == '__main__':
    main()