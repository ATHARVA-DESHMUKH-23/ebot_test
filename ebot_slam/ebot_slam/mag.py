import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
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
        import smbus
        import time

        self.bus = smbus.SMBus(0)
        self.addr = 0x1F

        # Check WHO_AM_I
        who = self.bus.read_byte_data(self.addr, 0x0D)
        if who != 0xC7:
            self.get_logger().error(f"FXOS8700 not detected! WHO_AM_I = {hex(who)}")
            raise RuntimeError("FXOS8700 not found")

        # Put into active mode
        self.bus.write_byte_data(self.addr, 0x2A, 0x00)  # standby
        self.bus.write_byte_data(self.addr, 0x5B, 0x1F)  # hybrid mode
        self.bus.write_byte_data(self.addr, 0x2A, 0x0D)  # active

       # Hard iron offsets
        self.mag_offset_x = 24.5
        self.mag_offset_y = -9.6
        self.mag_offset_z = 39.65

        # Soft iron scale
        self.mag_scale_x = 0.99
        self.mag_scale_y = 1.01
        self.mag_scale_z = 1.0


        self.timer = self.create_timer(0.1, self.read_sensor)  # 50 Hz

        # ---- yaw rate helpers ----
        self.last_yaw = None
        self.last_time = time.time()

        self.get_logger().info("FXOS8700 EKF-ready node started")

    def read_sensor(self):

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Read magnetometer registers
        data = self.bus.read_i2c_block_data(self.addr, 0x33, 6)

        mag_x = (data[0] << 8 | data[1])
        mag_y = (data[2] << 8 | data[3])
        mag_z = (data[4] << 8 | data[5])

        if mag_x > 32767: mag_x -= 65536
        if mag_y > 32767: mag_y -= 65536
        if mag_z > 32767: mag_z -= 65536

        # Scale (0.1 uT per LSB approx)
        mag_x *= 0.1
        mag_y *= 0.1
        mag_z *= 0.1

        # For roll/pitch you are already using MPU, so
        # if this node is magnetometer-only,
        # you can remove accel normalization here
        accel_x = 0.0
        accel_y = 0.0
        accel_z = 1.0
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
            0.0,   0.0,   0.5
        ]


        # Angular velocity (yaw rate)
        imu_msg.angular_velocity.z = 0.0
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
