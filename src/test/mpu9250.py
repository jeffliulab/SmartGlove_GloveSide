#!/usr/bin/env python3
"""
MPU9250传感器测试脚本 - 用于验证MPU9250是否正常工作
"""
import smbus 
import time
import math

# MPU9250的I2C地址（通常为0x68，如果AD0接高电平则为0x69）
MPU9250_ADDRESS = 0x68

# MPU9250寄存器地址
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_OUT_H   = 0x41
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
PWR_MGMT_1   = 0x6B
WHO_AM_I     = 0x75

def setup_mpu9250():
    """初始化MPU9250传感器"""
    # 创建I2C总线对象
    bus = smbus.SMBus(1)  # 树莓派的I2C总线编号通常为1
    
    # 检查设备ID
    try:
        who_am_i = bus.read_byte_data(MPU9250_ADDRESS, WHO_AM_I)
        print(f"设备ID: 0x{who_am_i:02X}")
        if who_am_i != 0x71 and who_am_i != 0x73:  # MPU9250的WHO_AM_I寄存器值通常为0x71或0x73
            print("警告: 设备ID不匹配，可能不是MPU9250")
    except Exception as e:
        print(f"无法读取设备ID: {e}")
        return None
    
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

def read_sensor_data(bus):
    """读取加速度计、陀螺仪和温度数据"""
    if bus is None:
        return None
    
    try:
        # 读取加速度数据
        accel_x = read_word_2c(bus, MPU9250_ADDRESS, ACCEL_XOUT_H)
        accel_y = read_word_2c(bus, MPU9250_ADDRESS, ACCEL_YOUT_H)
        accel_z = read_word_2c(bus, MPU9250_ADDRESS, ACCEL_ZOUT_H)
        
        # 转换为g值 (±2g范围)
        accel_scale = 16384.0  # 在±2g范围内，16384 LSB/g
        accel_x = accel_x / accel_scale
        accel_y = accel_y / accel_scale
        accel_z = accel_z / accel_scale
        
        # 读取陀螺仪数据
        gyro_x = read_word_2c(bus, MPU9250_ADDRESS, GYRO_XOUT_H)
        gyro_y = read_word_2c(bus, MPU9250_ADDRESS, GYRO_YOUT_H)
        gyro_z = read_word_2c(bus, MPU9250_ADDRESS, GYRO_ZOUT_H)
        
        # 转换为度/秒 (±250度/秒范围)
        gyro_scale = 131.0  # 在±250度/秒范围内，131 LSB/(度/秒)
        gyro_x = gyro_x / gyro_scale
        gyro_y = gyro_y / gyro_scale
        gyro_z = gyro_z / gyro_scale
        
        # 读取温度数据
        temp = read_word_2c(bus, MPU9250_ADDRESS, TEMP_OUT_H)
        temp = (temp / 340.0) + 36.53  # 来自MPU9250数据手册的温度公式
        
        return {
            'accel': {'x': accel_x, 'y': accel_y, 'z': accel_z},
            'gyro': {'x': gyro_x, 'y': gyro_y, 'z': gyro_z},
            'temp': temp
        }
    except Exception as e:
        print(f"读取传感器数据失败: {e}")
        return None

def calculate_tilt(accel_data):
    """计算倾斜角度（基于加速度计数据）"""
    x = accel_data['x']
    y = accel_data['y']
    z = accel_data['z']
    
    # 计算绕X轴和Y轴的倾斜角度（单位：度）
    roll = math.atan2(y, z) * 180.0 / math.pi
    pitch = math.atan2(-x, math.sqrt(y*y + z*z)) * 180.0 / math.pi
    
    return roll, pitch

def main():
    """主函数"""
    print("初始化MPU9250...")
    bus = setup_mpu9250()
    
    if bus is None:
        print("初始化失败，请检查连接和I2C配置")
        return
    
    print("MPU9250初始化成功！正在读取数据...")
    print("按Ctrl+C停止程序")
    
    try:
        while True:
            data = read_sensor_data(bus)
            if data:
                accel = data['accel']
                gyro = data['gyro']
                temp = data['temp']
                
                roll, pitch = calculate_tilt(accel)
                
                print("\n===== MPU9250 传感器数据 =====")
                print(f"加速度 (g):\tX: {accel['x']:.3f}\tY: {accel['y']:.3f}\tZ: {accel['z']:.3f}")
                print(f"陀螺仪 (°/s):\tX: {gyro['x']:.3f}\tY: {gyro['y']:.3f}\tZ: {gyro['z']:.3f}")
                print(f"温度: {temp:.2f} °C")
                print(f"倾斜角度:\tRoll: {roll:.2f}°\tPitch: {pitch:.2f}°")
            else:
                print("无法读取数据")
            
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n程序已停止")
    except Exception as e:
        print(f"发生错误: {e}")

if __name__ == "__main__":
    main()