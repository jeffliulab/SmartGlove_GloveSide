#!/usr/bin/env python3
"""
使用MPU9250检测抬起和降下动作

THIS SCRIPT DOESN"T WORK PROPERLY

SO DON"T USE IT NOW
"""
import smbus
import time
import math
import numpy as np
from collections import deque

# MPU9250的I2C地址
MPU9250_ADDRESS = 0x68

# MPU9250寄存器地址
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
PWR_MGMT_1   = 0x6B

# 动作检测参数
LIFT_THRESHOLD = 1.2     # 抬起阈值: >1.2g视为抬起动作开始
DROP_THRESHOLD = 0.8     # 降下阈值: <0.8g视为降下动作开始
MOTION_DURATION = 0.3    # 动作持续时间阈值(秒)
COOLDOWN_TIME = 0.5      # 两次检测之间的冷却时间(秒)

def setup_mpu9250():
    """初始化MPU9250传感器"""
    bus = smbus.SMBus(1)
    
    try:
        # 唤醒传感器
        bus.write_byte_data(MPU9250_ADDRESS, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        return bus
    except Exception as e:
        print(f"初始化MPU9250失败: {e}")
        return None

def read_word(bus, address, reg):
    """读取一个16位的字（两个字节）"""
    try:
        high = bus.read_byte_data(address, reg)
        low = bus.read_byte_data(address, reg + 1)
        value = (high << 8) + low
        return value
    except Exception as e:
        print(f"读取数据失败: {e}")
        return 0

def read_word_2c(bus, address, reg):
    """读取一个有符号的16位字"""
    val = read_word(bus, address, reg)
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

def read_accel_z(bus):
    """只读取Z轴加速度数据"""
    if bus is None:
        return None
    
    try:
        # 读取加速度数据
        accel_z = read_word_2c(bus, MPU9250_ADDRESS, ACCEL_ZOUT_H)
        
        # 转换为g值 (±2g范围)
        accel_scale = 16384.0  # 在±2g范围内，16384 LSB/g
        accel_z = accel_z / accel_scale
        
        return accel_z
    except Exception as e:
        print(f"读取Z轴加速度数据失败: {e}")
        return None

class MotionDetector:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.accel_z_history = deque(maxlen=window_size)
        self.last_detection_time = 0
        self.motion_start_time = 0
        self.is_lifting = False
        self.is_dropping = False
        self.motion_in_progress = False
    
    def update(self, accel_z):
        """更新加速度历史并检测动作"""
        current_time = time.time()
        self.accel_z_history.append(accel_z)
        
        # 如果数据点不足，或者在冷却期内，则不进行检测
        if len(self.accel_z_history) < self.window_size or \
           current_time - self.last_detection_time < COOLDOWN_TIME:
            return None
        
        # 使用平均值来平滑噪声
        avg_accel_z = sum(self.accel_z_history) / len(self.accel_z_history)
        
        # 动作检测逻辑
        if not self.motion_in_progress:
            # 检测动作开始
            if avg_accel_z > LIFT_THRESHOLD and not self.is_lifting:
                self.is_lifting = True
                self.motion_start_time = current_time
                self.motion_in_progress = True
                print("抬起动作开始")
            elif avg_accel_z < DROP_THRESHOLD and not self.is_dropping:
                self.is_dropping = True
                self.motion_start_time = current_time
                self.motion_in_progress = True
                print("降下动作开始")
        else:
            # 检测动作是否完成 - 当加速度恢复到正常范围
            if DROP_THRESHOLD < avg_accel_z < LIFT_THRESHOLD:
                motion_duration = current_time - self.motion_start_time
                if motion_duration > MOTION_DURATION:
                    if self.is_lifting:
                        print(f"检测到抬起动作! 持续时间: {motion_duration:.2f}秒")
                        self.is_lifting = False
                        self.last_detection_time = current_time
                        self.motion_in_progress = False
                        return "LIFT"
                    elif self.is_dropping:
                        print(f"检测到降下动作! 持续时间: {motion_duration:.2f}秒")
                        self.is_dropping = False
                        self.last_detection_time = current_time
                        self.motion_in_progress = False
                        return "DROP"
        
        return None

def main():
    """主函数"""
    print("初始化MPU9250...")
    bus = setup_mpu9250()
    
    if bus is None:
        print("初始化失败，请检查连接")
        return
    
    print("MPU9250初始化成功！正在监测动作...")
    print("按Ctrl+C停止程序")
    
    # 创建动作检测器
    detector = MotionDetector(window_size=5)  # 使用5个样本的窗口进行平滑处理
    
    # 用于数据可视化的简单方法
    last_values = []
    
    try:
        while True:
            accel_z = read_accel_z(bus)
            if accel_z is not None:
                motion = detector.update(accel_z)
                
                # 存储最近的值用于可视化
                last_values.append(accel_z)
                if len(last_values) > 20:
                    last_values.pop(0)
                
                # 简单的ASCII图表可视化
                bar = ""
                for val in last_values:
                    if val > LIFT_THRESHOLD:
                        bar += "▲"  # 抬起
                    elif val < DROP_THRESHOLD:
                        bar += "▼"  # 降下
                    else:
                        bar += "■"  # 正常
                
                print(f"\rZ加速度: {accel_z:.3f}g {bar}", end="")
                
                if motion:
                    print(f"\n检测到动作: {motion}")
            
            time.sleep(0.05)  # 20Hz采样率
    except KeyboardInterrupt:
        print("\n程序已停止")
    except Exception as e:
        print(f"\n发生错误: {e}")

if __name__ == "__main__":
    main()