import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelToArduino(Node):
    def __init__(self):
        super().__init__('cmdvel_to_arduino')

        self.serial = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.get_logger().info("EBot Serial Driver Started")

    def cmd_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        data = f"{v:.2f},{w:.2f}\n"
        self.serial.write(data.encode())

def main():
    rclpy.init()
    node = CmdVelToArduino()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()