#!/usr/bin/env python3

# ROS Core
import rclpy
from rclpy.node import Node

# Messages interface
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature

# General libraries
import serial
import re
import math


class ImuRawDataPublisher(Node):
    def __init__(self):
        super().__init__("imu_raw_data_publisher")

        # Publisher parameters
        self.imu_pub_ = self.create_publisher(Imu, '/imu/raw_data', 10)
        self.temperature_pub_ = self.create_publisher(Temperature, '/imu/temperature', 10)

        # Timer parameters
        self.freq = 10    # 10hz
        self.timer_ = self.create_timer(self.freq, self.imu_publisher_cb)

        # Serial reader parameters
        self.conn_ = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)


    def imu_publisher_cb(self):
        received_data = self.conn_.readline()

        if received_data:    # is valid
            received_data = received_data.decode('utf-8', errors='replace').strip()

            imu_data = self.parse_imu_data(received_data)

            if imu_data is None:
                return

            imu_msg = Imu()
            temp_msg = Temperature()

            # ========= TIME AND FRAME =========
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            temp_msg.header.stamp = self.get_clock().now().to_msg()
            temp_msg.header.frame_id = 'imu_link'

            # ========= ORIENTATION =========
            imu_msg.orientation_covariance[0] = 0.001
            imu_msg.orientation_covariance[1] = 0.001
            imu_msg.orientation_covariance[2] = 0.001

            # ========= ACC =========
            imu_msg.linear_acceleration.x = imu_data[0][0] 
            imu_msg.linear_acceleration.y = imu_data[0][1] 
            imu_msg.linear_acceleration.z = imu_data[0][2] 

            imu_msg.linear_acceleration_covariance[0] = 0.01
            imu_msg.linear_acceleration_covariance[4] = 0.01
            imu_msg.linear_acceleration_covariance[8] = 0.01

            # ========= GYRO =========
            imu_msg.angular_velocity.x = imu_data[1][0] 
            imu_msg.angular_velocity.y = imu_data[1][1] 
            imu_msg.angular_velocity.z = imu_data[1][2] 

            imu_msg.angular_velocity_covariance[0] = 0.01
            imu_msg.angular_velocity_covariance[4] = 0.01
            imu_msg.angular_velocity_covariance[8] = 0.01

            self.imu_pub_.publish(imu_msg)
            self.temperature_pub_.publish(temp_msg)


    def parse_imu_data(self, received_msg: str, acc_sens = 16384.0, gyro_sens = 131.0):

        pattern = r"Acc:\s*\[([^\]]+)\]*Gyr:\s*\[([^\]]+)\]*Tmp:\s*\[([^\]]+)\]"
        
        match = re.search(pattern, received_msg)
        if not match:
            return None

        acc_raw = list(map(int, match.group(1).split(',')))
        gyro_raw = list(map(int, match.group(2).split(',')))
        tmp_raw = int(match.group(2))

        # ========= ACC =========
        acc_g = [v / acc_sens for v in acc_raw]               # g
        acc_ms2 = [v * 9.80665 for v in acc_g]                # m/s^2

        # ========= GYRO =========
        gyr_dps = [v / gyro_sens for v in gyro_raw]           # deg/s
        gyr_rads = [v * (math.pi / 180.0) for v in gyr_dps] # rad/s

        # ========= TEMPERATURE =========
        temp_c = ((tmp_raw - 21.0) / 333.87) + 21.0

        return [acc_ms2, gyr_rads, temp_c]


def main(args=None):
    try:
        rclpy.init(args=args)
        
        node = ImuRawDataPublisher()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
