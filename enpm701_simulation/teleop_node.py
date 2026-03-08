import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

LINEAR_SPEED = 0.5
ANGULAR_SPEED = 2.0

HELP = """
Controls:
  W / S   : forward / backward
  A / D   : turn left / right
  Space   : stop
  G       : toggle gripper open/close
  L       : toggle gripper lift up/down
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
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self._grip_pub = self.create_publisher(Twist, 'gripper_vel', 1)
        self._lift_pub = self.create_publisher(Twist, 'gripper_vel', 1)

        self._grip_closed = False
        self._lift_up = False

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_vel_pub.publish(msg)

    def publish_grip(self):
        msg = Twist()
        msg.linear.x = 0.03 if self._grip_closed else 0.0
        msg.linear.y = 0.005 if self._lift_up else 0.0
        self._grip_pub.publish(msg)

    def publish_lift(self):
        msg = Twist()
        msg.linear.x = 0.03 if self._grip_closed else 0.0
        msg.linear.y = 0.005 if self._lift_up else 0.0
        self._lift_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    settings = termios.tcgetattr(sys.stdin)

    print(HELP)

    try:
        while rclpy.ok():
            key = get_key(settings)

            linear = 0.0
            angular = 0.0

            if key == 'w':
                linear = LINEAR_SPEED
            elif key == 's':
                linear = -LINEAR_SPEED
            elif key == 'a':
                angular = ANGULAR_SPEED
            elif key == 'd':
                angular = -ANGULAR_SPEED
            elif key == ' ':
                pass  # stop — zero twist published below
            elif key == 'g':
                node._grip_closed = not node._grip_closed
                node.publish_grip()
                print(f"Gripper: {'closed' if node._grip_closed else 'open'}")
                continue
            elif key == 'l':
                node._lift_up = not node._lift_up
                node.publish_lift()
                print(f"Lift: {'up' if node._lift_up else 'down'}")
                continue
            elif key == '\x03':  # Ctrl+C
                break
            else:
                continue

            node.publish_twist(linear, angular)

    finally:
        node.publish_twist(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
