import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import board
import busio
import adafruit_fxos8700
import math
from tf_transformations import quaternion_from_euler
import time


class FXOS8700Node(Node):

    def __init__(self):
        super().__init__('fxos8700_node')

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/fxos8700/imu', 10)
        self.yaw_pub = self.create_publisher(Float32, '/fxos8700/yaw', 10)

        # I2C
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_fxos8700.FXOS8700(i2c, address=0x1F)

        # Hard iron offsets
        self.mag_offset_x = 12.4
        self.mag_offset_y = -16.25
        self.mag_offset_z = 33.65

        # Soft iron scale
        self.mag_scale_x = 0.98
        self.mag_scale_y = 1.02
        self.mag_scale_z = 1.0




        self.timer = self.create_timer(0.02, self.read_sensor)  # 50 Hz

        # ---- yaw rate helpers ----
        self.last_yaw = None
        self.last_time = time.time()

        self.get_logger().info("FXOS8700 EKF-ready node started")

    def read_sensor(self):

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        accel_x, accel_y, accel_z = self.sensor.accelerometer
        mag_x, mag_y, mag_z = self.sensor.magnetometer

        # Apply hard iron offsets
        # Hard iron
        mag_x -= self.mag_offset_x
        mag_y -= self.mag_offset_y
        mag_z -= self.mag_offset_z

        # Soft iron
        mag_x *= self.mag_scale_x
        mag_y *= self.mag_scale_y
        mag_z *= self.mag_scale_z


        # Normalize acceleration
        norm = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        if norm == 0:
            return

        accel_x /= norm
        accel_y /= norm
        accel_z /= norm

        # Roll & Pitch
        roll = math.atan2(accel_y, accel_z)
        pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))

        # Tilt compensation
        mag_x_comp = mag_x * math.cos(pitch) + mag_z * math.sin(pitch)
        mag_y_comp = (mag_x * math.sin(roll) * math.sin(pitch) +
                      mag_y * math.cos(roll) -
                      mag_z * math.sin(roll) * math.cos(pitch))

        # ---- YAW ----
        yaw = -math.atan2(mag_y_comp, mag_x_comp)

        # normalize for EKF continuity
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        # ---- YAW RATE ----
        yaw_rate = 0.0
        if self.last_yaw is not None and dt > 0:
            dyaw = yaw - self.last_yaw
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
            yaw_rate = dyaw / dt

        self.last_yaw = yaw

        # Quaternion
        q = quaternion_from_euler(roll, pitch, yaw)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Orientation
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        imu_msg.orientation_covariance = [
            999.0, 0.0,   0.0,
            0.0,   999.0, 0.0,
            0.0,   0.0,   0.02
        ]


        # Angular velocity (yaw rate)
        imu_msg.angular_velocity.z = yaw_rate
        imu_msg.angular_velocity_covariance = [
            -1.0, 0.0, 0.0,
             0.0, -1.0, 0.0,
             0.0, 0.0, 0.05
        ]

        # Linear acceleration
        imu_msg.linear_acceleration.x = accel_x * 9.81
        imu_msg.linear_acceleration.y = accel_y * 9.81
        imu_msg.linear_acceleration.z = accel_z * 9.81

        self.imu_pub.publish(imu_msg)

        # ---- YAW topic (unchanged) ----
        yaw_deg = math.degrees(yaw)
        if yaw_deg < 0:
            yaw_deg += 360

        yaw_msg = Float32()
        yaw_msg.data = float(yaw_deg)
        self.yaw_pub.publish(yaw_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FXOS8700Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
