import threading
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

HELP = """
Controls:
  M     : cycle mode (red -> green -> blue)
  Q/A   : lower bound H up/down
  W/S   : lower bound S up/down
  E/D   : lower bound V up/down
  R/F   : upper bound H up/down
  T/G   : upper bound S up/down
  Y/H   : upper bound V up/down
  (red mode only - second range)
  U/J   : lower_2 H up/down
  I/K   : lower_2 S up/down
  O/L   : lower_2 V up/down
  Z/X   : upper_2 H up/down
  C/V   : upper_2 S up/down
  B/N   : upper_2 V up/down
  Ctrl+C: quit
"""

BOUNDS = {
    'red':   {'lower': np.array([171, 138, 47]), 'upper': np.array([255, 255, 255]),
              'lower2': np.array([0, 175, 193]),  'upper2': np.array([46, 255, 255])},
    'green': {'lower': np.array([39, 0, 10]),     'upper': np.array([92, 255, 255])},
    'blue':  {'lower': np.array([68, 0, 10]),     'upper': np.array([130, 255, 255])},
}

mode = 'red'


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_bounds():
    b = BOUNDS[mode]
    print(f"[{mode}] lower={b['lower']} upper={b['upper']}", end='')
    if mode == 'red':
        print(f"  lower2={b['lower2']} upper2={b['upper2']}", end='')
    print()


class ColorPicker(Node):
    def __init__(self):
        super().__init__('colorpicker_node')
        self._image_pub = self.create_publisher(Image, 'colorpicker_image', 1)
        self._bridge = CvBridge()

        self._camera = cv2.VideoCapture(0)
        self._camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.create_timer(0.1, self._timer_callback)

    def _timer_callback(self):
        ret, cv_img = self._camera.read()
        if not ret:
            return
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        b = BOUNDS[mode]

        if mode == 'red':
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, b['lower'], b['upper']),
                cv2.inRange(hsv, b['lower2'], b['upper2'])
            )
        else:
            mask = cv2.inRange(hsv, b['lower'], b['upper'])

        result = cv2.bitwise_and(cv_img, cv_img, mask=mask)
        cv2.putText(result, f"mode: {mode}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        self._image_pub.publish(self._bridge.cv2_to_imgmsg(result))

    def destroy_node(self):
        self._camera.release()
        super().destroy_node()


def main(args=None):
    global mode
    rclpy.init(args=args)
    node = ColorPicker()
    settings = termios.tcgetattr(sys.stdin)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(HELP)
    print_bounds()

    KEY_ACTIONS = {
        'q': ('lower', 0,  1), 'a': ('lower', 0, -1),
        'w': ('lower', 1,  1), 's': ('lower', 1, -1),
        'e': ('lower', 2,  1), 'd': ('lower', 2, -1),
        'r': ('upper', 0,  1), 'f': ('upper', 0, -1),
        't': ('upper', 1,  1), 'g': ('upper', 1, -1),
        'y': ('upper', 2,  1), 'h': ('upper', 2, -1),
    }
    RED_ONLY_ACTIONS = {
        'u': ('lower2', 0,  1), 'j': ('lower2', 0, -1),
        'i': ('lower2', 1,  1), 'k': ('lower2', 1, -1),
        'o': ('lower2', 2,  1), 'l': ('lower2', 2, -1),
        'z': ('upper2', 0,  1), 'x': ('upper2', 0, -1),
        'c': ('upper2', 1,  1), 'v': ('upper2', 1, -1),
        'b': ('upper2', 2,  1), 'n': ('upper2', 2, -1),
    }

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key == '\x03':
                break
            elif key == 'm':
                mode = {'red': 'green', 'green': 'blue', 'blue': 'red'}[mode]
                print_bounds()
            elif key in KEY_ACTIONS:
                field, idx, delta = KEY_ACTIONS[key]
                BOUNDS[mode][field][idx] = int(
                    np.clip(BOUNDS[mode][field][idx] + delta, 0, 255))
                print_bounds()
            elif key in RED_ONLY_ACTIONS and mode == 'red':
                field, idx, delta = RED_ONLY_ACTIONS[key]
                BOUNDS['red'][field][idx] = int(
                    np.clip(BOUNDS['red'][field][idx] + delta, 0, 255))
                print_bounds()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
