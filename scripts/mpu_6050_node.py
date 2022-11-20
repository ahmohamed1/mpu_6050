#! /usr/bin/env python3

import adafruit_mpu6050
import board
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import (Quaternion, Vector3)
from sensor_msgs.msg import Temperature, Imu

class MPU6050Node(Node):
    def __init__(self):
        super().__init__("mpu6050")
        self.i2c = board.I2C()
        self.mpu = adafruit_mpu6050.MPU6050(self.i2c)
        self.imu_frame = self.declare_parameter("~imu_frame", 'imu_link').value
        self.publisher_ = self.create_publisher(Imu, "imu", 15)
        self.frequency = 98
        self.timer_ = self.create_timer((1 / self.frequency), self.mpu_read)
        self.get_logger().info("MP6050 stream opened.")

    def read_mpu6050(self):
        msg = Float32MultiArray()
        msg.data = [round(self.mpu.acceleration[0], 2),
                    round(self.mpu.acceleration[1], 2),
                    round(self.mpu.acceleration[2], 2)]
        # to read gyro mpu.gyro
        self.publisher_.publish(msg)

    def mpu_read(self):
        
        imu_msg = Imu()

        # Read gyroscope values.
        # At default sensitivity of 250deg/s we need to scale by 131.
        gyro = Vector3()
        gyro.x = self.mpu.gyro[0]
        gyro.y = self.mpu.gyro[1]
        gyro.z = self.mpu.gyro[2]
        
        accel = Vector3()
        accel.x = self.mpu.acceleration[0]
        accel.y = self.mpu.acceleration[1]
        accel.z = self.mpu.acceleration[2]
        imu_msg.angular_velocity = gyro
        imu_msg.linear_acceleration = accel
        # add header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame
        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()