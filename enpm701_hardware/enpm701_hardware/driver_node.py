import rclpy
from rclpy.node import Node
import pigpio
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# GPIO pins
LF = 6   # left forward
LB = 13  # left backward
RB = 19  # right backward
RF = 26  # right forward
GR = 16  # gripper

PWM_FREQ = 50
# DRIVE_DUTY = int(round(0.5 * 255))        # 50% duty cycle for driving
GRIPPER_OPEN = int(round(0.085 * 255))    # ~22 — tune if needed
GRIPPER_CLOSED = int(round(0.045 * 255))   # ~13 — tune if needed

class DriverNode(Node):
    def __init__(self):
        super().__init__('driver_node')
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError('Could not connect to pigpio daemon. Is pigpiod running?')
        for pin in (LF, LB, RB, RF, GR):
            self._pi.set_mode(pin, pigpio.OUTPUT)
            self._pi.set_PWM_frequency(pin, PWM_FREQ)
            self._pi.set_PWM_dutycycle(pin, 0)
        # Start with gripper open
        self._pi.set_PWM_dutycycle(GR, GRIPPER_OPEN)
        self._grip_closed = False

        self._cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, 10)
        self._cmd_grip_sub = self.create_subscription(Bool, 'gripper_cmd', self._cmd_grip_callback, 10)




    def _cmd_vel_callback(self, msg):
        linear = max(min(msg.linear.x, 1.0), -1.0)
        angular = max(min(msg.angular.z, 1.0), -1.0)
        self._drive(linear, angular)

    def _cmd_grip_callback(self, msg):
        if msg.data != self._grip_closed:
            self._grip_closed = msg.data
            duty = GRIPPER_CLOSED if self._grip_closed else GRIPPER_OPEN
            self._pi.set_PWM_dutycycle(GR, duty)

    def _drive(self, linear, angular):
        """
        linear  > 0 : forward
        linear  < 0 : backward
        angular > 0 : turn left  (right wheel forward, left wheel back)
        angular < 0 : turn right (left wheel forward, right wheel back)
        """
        lf = lb = rf = rb = 0

        if linear > 0:
            lf = rf = int(round(linear * 255))
        elif linear < 0:
            lb = rb = int(round(-linear * 255))
        elif angular > 0:   # turn left in place
            rf = int(round(angular * 255))
            lb = int(round(angular * 255))
        elif angular < 0:   # turn right in place
            lf = int(round(-angular * 255))
            rb = int(round(-angular * 255))

        self._pi.set_PWM_dutycycle(LF, lf)
        self._pi.set_PWM_dutycycle(LB, lb)
        self._pi.set_PWM_dutycycle(RB, rb)
        self._pi.set_PWM_dutycycle(RF, rf)

    def shutdown(self):
        for pin in (LF, LB, RB, RF):
            self._pi.set_PWM_dutycycle(pin, 0)
        self._pi.set_PWM_dutycycle(GR, GRIPPER_OPEN)
        self._pi.stop()

def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
