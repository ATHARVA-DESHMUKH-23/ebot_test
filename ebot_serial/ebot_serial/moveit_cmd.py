import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup

class MoveItCommander(Node):
    def __init__(self):
        super().__init__('moveit_commander')
        self.client = ActionClient(self, MoveGroup, '/move_action')

    def send_goal(self):
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 1
        goal.request.allowed_planning_time = 5.0

        self.client.wait_for_server()
        self.client.send_goal_async(goal)

def main():
    rclpy.init()
    node = MoveItCommander()
    node.send_goal()
    rclpy.spin_once(node, timeout_sec=2)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
