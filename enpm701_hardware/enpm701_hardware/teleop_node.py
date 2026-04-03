import sys
import tty
import termios
import rclpy
from rclpy.node import Node
import pigpio
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist



HELP = """
Controls:
  W / S   : forward / backward
  A / D   : turn left / right
  Space   : stop
  G       : toggle gripper open/close
  Ctrl+C  : quit
"""


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._cmd_grip_pub = self.create_publisher(Bool, 'gripper_cmd', 10)
        self._grip_closed = False


    def drive(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_vel_pub.publish(msg)

    def stop(self):
        msg = Twist()
        grip_msg = Bool()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        grip_msg.data = False
        self._cmd_vel_pub.publish(msg)
        self._cmd_grip_pub.publish(grip_msg)

    def toggle_gripper(self):
        self._grip_closed = not self._grip_closed
        msg = Bool()
        msg.data = self._grip_closed
        self._cmd_grip_pub.publish(msg)

    def shutdown(self):
        self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    settings = termios.tcgetattr(sys.stdin)

    print(HELP)

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key == 'w':
                node.drive(0.5, 0)
            elif key == 's':
                node.drive(-0.5, 0)
            elif key == 'a':
                node.drive(0, 0.5)
            elif key == 'd':
                node.drive(0, -0.5)
            elif key == ' ':
                node.stop()
            elif key == 'g':
                node.toggle_gripper()
                print(f"Gripper: {'closed' if node._grip_closed else 'open'}")
            elif key == '\x03':  # Ctrl+C
                break

    finally:
        node.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
