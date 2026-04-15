import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import serial



class CmdVelToArduino(Node):


    def __init__(self):
        super().__init__('cmdvel_to_arduino')

        # -----------------------------
        # Declare parameters
        # -----------------------------
        self.declare_parameter('serial_port', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # -----------------------------
        # Serial connection
        # -----------------------------
        try:
            self.serial = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            raise e

        # store latest values
        self.v = 0.0
        self.w = 0.0
        self.bot_yaw = 0.0

        # subscribers
        # store latest values
        self.v = 0.0
        self.w = 0.0
        self.bot_yaw = 0.0

        # subscribers
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/ebot_info/yaw',
            self.yaw_callback,
            10
        )

        # timer → 20 Hz serial transmission
        self.timer = self.create_timer(0.05, self.send_serial)

        self.get_logger().info("EBot Serial Driver Started")



    def cmd_callback(self, msg):
        self.v = -msg.linear.x
        self.w = -msg.angular.z


    def yaw_callback(self, msg):
        self.bot_yaw = msg.data


    def send_serial(self):

        data = f"{self.v:.2f},{self.w:.2f},{self.bot_yaw:.2f}\n"
        # print(f"Sending to Arduino: {data.strip()}")

        try:
            self.serial.write(data.encode())
        except:
            self.get_logger().warn("Serial write failed")


def main():
    rclpy.init()
    node = CmdVelToArduino()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()