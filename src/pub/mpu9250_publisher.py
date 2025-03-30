#!/usr/bin/env python3
"""
MPU9250 ROS2 简单发布者 - 无需创建包，直接运行此脚本
将MPU9250传感器数据发布到ROS2话题
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, String
import smbus
import time
import math
import numpy as np
import json
import struct
from scipy.spatial.transform import Rotation

class MPU9250Publisher(Node):
    def __init__(self):
        super().__init__('mpu9250_publisher')
        
        # MPU9250的I2C地址
        self.MPU9250_ADDRESS = 0x68
        
        # MPU9250寄存器地址
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.TEMP_OUT_H   = 0x41
        self.GYRO_XOUT_H  = 0x43
        self.GYRO_YOUT_H  = 0x45
        self.GYRO_ZOUT_H  = 0x47
        self.PWR_MGMT_1   = 0x6B
        self.WHO_AM_I     = 0x75
        
        # 初始化I2C总线
        self.get_logger().info('INITIALIZE I2C BUS')
        try:
            self.bus = smbus.SMBus(1)  # 树莓派的I2C总线号通常为1
            
            # 检查设备ID
            who_am_i = self.bus.read_byte_data(self.MPU9250_ADDRESS, self.WHO_AM_I)
            self.get_logger().info(f'DEVICE ID: 0x{who_am_i:02X}')
            
            # 唤醒传感器
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.PWR_MGMT_1, 0x00)
            time.sleep(0.1)
            
            # 设置陀螺仪量程为±250度/秒
            self.bus.write_byte_data(self.MPU9250_ADDRESS, 0x1B, 0x00)
            
            # 设置加速度计量程为±2g
            self.bus.write_byte_data(self.MPU9250_ADDRESS, 0x1C, 0x00)
            
        except Exception as e:
            self.get_logger().error(f'INITIALIZE MPU9250 FAILED: {e}')
            raise e
        
        # 创建发布者
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.temp_pub = self.create_publisher(Temperature, 'imu/temperature', 10)
        self.accel_pub = self.create_publisher(Vector3, 'imu/acceleration', 10)
        self.gyro_pub = self.create_publisher(Vector3, 'imu/angular_velocity', 10)
        self.json_pub = self.create_publisher(String, 'imu/all_data', 10)
        
        # 姿态估计变量
        self.last_time = time.time()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.alpha = 0.98  # 互补滤波器系数
        
        # 创建定时器，50Hz的数据发布频率
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info('MPU9250 PUBLISHER START SUCCESSFULLY')
    
    def read_word(self, reg):
        """读取一个16位的字（两个字节）"""
        try:
            high = self.bus.read_byte_data(self.MPU9250_ADDRESS, reg)
            low = self.bus.read_byte_data(self.MPU9250_ADDRESS, reg + 1)
            value = (high << 8) + low
            return value
        except Exception as e:
            self.get_logger().error(f'LOAD DATA FAILED: {e}')
            return 0
    
    def read_word_2c(self, reg):
        """读取一个有符号的16位字"""
        val = self.read_word(reg)
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val
    
    def read_sensor_data(self):
        """读取MPU9250的传感器数据"""
        try:
            # 读取加速度数据
            accel_x = self.read_word_2c(self.ACCEL_XOUT_H)
            accel_y = self.read_word_2c(self.ACCEL_YOUT_H)
            accel_z = self.read_word_2c(self.ACCEL_ZOUT_H)
            
            # 转换为g值 (±2g范围)
            accel_scale = 16384.0  # 在±2g范围内，16384 LSB/g
            accel_x = accel_x / accel_scale
            accel_y = accel_y / accel_scale
            accel_z = accel_z / accel_scale
            
            # 读取陀螺仪数据
            gyro_x = self.read_word_2c(self.GYRO_XOUT_H)
            gyro_y = self.read_word_2c(self.GYRO_YOUT_H)
            gyro_z = self.read_word_2c(self.GYRO_ZOUT_H)
            
            # 转换为弧度/秒 (±250度/秒范围)
            gyro_scale = 131.0  # 在±250度/秒范围内，131 LSB/(度/秒)
            gyro_x_deg = gyro_x / gyro_scale
            gyro_y_deg = gyro_y / gyro_scale
            gyro_z_deg = gyro_z / gyro_scale
            
            # 转换为弧度/秒
            gyro_x = math.radians(gyro_x_deg)
            gyro_y = math.radians(gyro_y_deg)
            gyro_z = math.radians(gyro_z_deg)
            
            # 读取温度数据
            temp = self.read_word_2c(self.TEMP_OUT_H)
            temp = (temp / 340.0) + 36.53  # 来自MPU9250数据手册的温度公式
            
            return {
                'accel': {'x': accel_x, 'y': accel_y, 'z': accel_z},
                'gyro': {'x': gyro_x, 'y': gyro_y, 'z': gyro_z},
                'gyro_deg': {'x': gyro_x_deg, 'y': gyro_y_deg, 'z': gyro_z_deg},
                'temp': temp
            }
        except Exception as e:
            self.get_logger().error(f'LOAD DATA FAILED: {e}')
            return None
    
    def update_orientation(self, accel, gyro):
        """使用互补滤波器更新姿态估计"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 从加速度计计算重力方向
        roll_acc = math.atan2(accel['y'], accel['z'])
        pitch_acc = math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2))
        
        # 整合陀螺仪数据
        self.roll = self.alpha * (self.roll + gyro['x'] * dt) + (1 - self.alpha) * roll_acc
        self.pitch = self.alpha * (self.pitch + gyro['y'] * dt) + (1 - self.alpha) * pitch_acc
        
        # 由于缺乏磁力计，偏航角只能通过陀螺仪积分来更新，会有漂移
        self.yaw += gyro['z'] * dt
        
        return self.roll, self.pitch, self.yaw
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """将欧拉角转换为四元数"""
        r = Rotation.from_euler('xyz', [roll, pitch, yaw])
        return r.as_quat()  # 返回[x, y, z, w]格式的四元数
    
    def timer_callback(self):
        """定时器回调函数，读取并发布传感器数据"""
        data = self.read_sensor_data()
        if data is None:
            return
        
        # 更新姿态估计
        roll, pitch, yaw = self.update_orientation(data['accel'], data['gyro'])
        
        # 将欧拉角转换为四元数
        quat = self.euler_to_quaternion(roll, pitch, yaw)
        
        # 创建并发布IMU消息
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        # 设置加速度数据 (m/s^2)
        imu_msg.linear_acceleration.x = data['accel']['x'] * 9.80665
        imu_msg.linear_acceleration.y = data['accel']['y'] * 9.80665
        imu_msg.linear_acceleration.z = data['accel']['z'] * 9.80665
        
        # 设置陀螺仪数据 (rad/s)
        imu_msg.angular_velocity.x = data['gyro']['x']
        imu_msg.angular_velocity.y = data['gyro']['y']
        imu_msg.angular_velocity.z = data['gyro']['z']
        
        # 设置方向四元数
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]
        
        # 发布IMU消息
        self.imu_pub.publish(imu_msg)
        
        # 发布温度消息
        temp_msg = Temperature()
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        temp_msg.header.frame_id = "imu_link"
        temp_msg.temperature = data['temp']
        self.temp_pub.publish(temp_msg)
        
        # 发布加速度向量
        accel_msg = Vector3()
        accel_msg.x = data['accel']['x']
        accel_msg.y = data['accel']['y']
        accel_msg.z = data['accel']['z']
        self.accel_pub.publish(accel_msg)
        
        # 发布角速度向量
        gyro_msg = Vector3()
        gyro_msg.x = data['gyro']['x']
        gyro_msg.y = data['gyro']['y']
        gyro_msg.z = data['gyro']['z']
        self.gyro_pub.publish(gyro_msg)
        
        # 将所有数据合并为一个JSON字符串并发布
        all_data = {
            'timestamp': time.time(),
            'accel': {
                'x': data['accel']['x'],
                'y': data['accel']['y'],
                'z': data['accel']['z']
            },
            'gyro': {
                'x': data['gyro_deg']['x'],  # 使用度/秒便于阅读
                'y': data['gyro_deg']['y'],
                'z': data['gyro_deg']['z']
            },
            'euler': {
                'roll': math.degrees(roll),  # 使用度便于阅读
                'pitch': math.degrees(pitch),
                'yaw': math.degrees(yaw)
            },
            'quaternion': {
                'x': quat[0],
                'y': quat[1],
                'z': quat[2],
                'w': quat[3]
            },
            'temperature': data['temp']
        }
        
        json_msg = String()
        json_msg.data = json.dumps(all_data)
        self.json_pub.publish(json_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MPU9250Publisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('USER INTERUPT, CLOSING...')
    except Exception as e:
        print(f'ERROR: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
