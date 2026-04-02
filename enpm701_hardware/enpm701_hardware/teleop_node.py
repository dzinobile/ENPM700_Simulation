import sys
import tty
import termios
import rclpy
from rclpy.node import Node
import pigpio

# GPIO pins
LF = 6   # left forward
LB = 13  # left backward
RB = 19  # right backward
RF = 26  # right forward
GR = 16  # gripper

PWM_FREQ = 50
DRIVE_DUTY = int(round(0.5 * 255))        # 50% duty cycle for driving
GRIPPER_OPEN = int(round(0.085 * 255))    # ~22 — tune if needed
GRIPPER_CLOSED = int(round(0.045 * 255))   # ~13 — tune if needed

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

    def drive(self, linear, angular):
        """
        linear  > 0 : forward
        linear  < 0 : backward
        angular > 0 : turn left  (right wheel forward, left wheel back)
        angular < 0 : turn right (left wheel forward, right wheel back)
        """
        lf = lb = rf = rb = 0

        if linear > 0:
            lf = rf = DRIVE_DUTY
        elif linear < 0:
            lb = rb = DRIVE_DUTY
        elif angular > 0:   # turn left in place
            rf = DRIVE_DUTY
            lb = DRIVE_DUTY
        elif angular < 0:   # turn right in place
            lf = DRIVE_DUTY
            rb = DRIVE_DUTY

        self._pi.set_PWM_dutycycle(LF, lf)
        self._pi.set_PWM_dutycycle(LB, lb)
        self._pi.set_PWM_dutycycle(RB, rb)
        self._pi.set_PWM_dutycycle(RF, rf)

    def stop(self):
        for pin in (LF, LB, RB, RF):
            self._pi.set_PWM_dutycycle(pin, 0)

    def toggle_gripper(self):
        self._grip_closed = not self._grip_closed
        duty = GRIPPER_CLOSED if self._grip_closed else GRIPPER_OPEN
        self._pi.set_PWM_dutycycle(GR, duty)

    def shutdown(self):
        self.stop()
        self._pi.set_PWM_dutycycle(GR, GRIPPER_OPEN)
        self._pi.stop()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    settings = termios.tcgetattr(sys.stdin)

    print(HELP)

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key == 'w':
                node.drive(1, 0)
            elif key == 's':
                node.drive(-1, 0)
            elif key == 'a':
                node.drive(0, 1)
            elif key == 'd':
                node.drive(0, -1)
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
