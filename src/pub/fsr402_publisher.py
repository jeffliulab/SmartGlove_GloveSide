#!/usr/bin/env python3
"""
FSR402 ROS2 发布者
通过串口读取 FSR402 的数据（打印格式为 "Force Sensor:  62767"），
提取出数字部分，并发布到 'force_sensor' 话题上。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import threading
import time
import re

class FSR402Publisher(Node):
    def __init__(self):
        super().__init__('fsr402_publisher')
        
        # 尝试打开串口设备
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("成功打开串口 /dev/ttyACM0")
        except Exception as e:
            self.get_logger().error("打开串口失败: " + str(e))
            raise e
        
        # 创建发布者，发布到 'force_sensor' 话题
        self.publisher_ = self.create_publisher(Int32, 'force_sensor', 10)
        
        # 启动一个独立线程用于读取串口数据
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True  # 主线程退出时，此线程也会退出
        self.serial_thread.start()
        
        self.get_logger().info("FSR402 发布者已启动。")
    
    def serial_callback(self, line: str):
        """
        串口数据回调函数：
        处理读取到的一行数据，提取出数字部分，并发布到 'force_sensor' 话题。
        预期数据格式: "Force Sensor:  62767"
        """
        # 使用正则表达式提取数字
        match = re.search(r'Force Sensor:\s*(\d+)', line)
        if match:
            sensor_value_str = match.group(1)
            try:
                sensor_value = int(sensor_value_str)
                msg = Int32()
                msg.data = sensor_value
                self.publisher_.publish(msg)
                self.get_logger().debug("发布的传感器数值: " + str(sensor_value))
            except ValueError as e:
                self.get_logger().error("数字转换失败: " + sensor_value_str)
        else:
            self.get_logger().warning("收到的行格式不符: " + line)
    
    def read_serial(self):
        """
        在一个独立线程中循环读取串口数据，
        每当读到有效数据时调用 serial_callback 处理。
        """
        while rclpy.ok():
            try:
                # 读取一行数据（等待时间由 timeout 控制）
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    self.get_logger().info("Reiceive: " + line)
                    self.serial_callback(line)
            except Exception as e:
                self.get_logger().error("读取串口数据错误: " + str(e))
                time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = FSR402Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，关闭 FSR402 发布者")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
