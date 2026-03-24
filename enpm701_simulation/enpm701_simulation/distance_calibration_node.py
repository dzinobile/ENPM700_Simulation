import csv
import datetime
import os
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
from ament_index_python.packages import get_package_share_directory

HELP = """
Controls:
  M     : cycle mode (red -> green -> blue)
  Space : log current color / distance / rect_h to CSV, advance distance
  R     : reset distance back to 0.3 m
  Ctrl+C: quit (saves timestamped copy of CSV)
"""

BOUNDS = {
    'red':   {'lower': np.array([171, 145, 47]), 'upper': np.array([255, 255, 255]),
              'lower2': np.array([0, 175, 193]),  'upper2': np.array([10, 255, 255])},
    'green': {'lower': np.array([39, 0, 10]),     'upper': np.array([92, 255, 255])},
    'blue':  {'lower': np.array([100, 109, 5]),     'upper': np.array([130, 255, 255])},
}

mode = 'red'
distance = 0.3
_data_rows = []


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def _csv_path():
    pkg = get_package_share_directory('enpm701_simulation')
    return os.path.join(pkg, 'block_distance_testing.csv')


def _build_rows():
    b = BOUNDS
    rows = [
        ['bound', 'H', 'S', 'V'],
        ['red lower 1',  *b['red']['lower'].tolist()],
        ['red upper 1',  *b['red']['upper'].tolist()],
        ['red lower 2',  *b['red']['lower2'].tolist()],
        ['red upper 2',  *b['red']['upper2'].tolist()],
        ['green lower',  *b['green']['lower'].tolist()],
        ['green upper',  *b['green']['upper'].tolist()],
        ['blue lower',   *b['blue']['lower'].tolist()],
        ['blue upper',   *b['blue']['upper'].tolist()],
        [],
        ['color', 'distance [m]', 'h [pixels]'],
    ]
    rows.extend(_data_rows)
    return rows


def _write_csv(path):
    with open(path, 'w', newline='') as f:
        csv.writer(f).writerows(_build_rows())


class DistanceCalibrationNode(Node):
    def __init__(self):
        super().__init__('distance_calibration_node')
        self._image_pub = self.create_publisher(Image, 'boundingboxes_image', 1)
        self._image_sub = self.create_subscription(
            Image, 'my_robot/camera/image_color', self._callback, 1)
        self._bridge = CvBridge()
        self.last_rect_h = None

    def _callback(self, ros_img):
        cv_img = self._bridge.imgmsg_to_cv2(ros_img)
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        b = BOUNDS[mode]

        if mode == 'red':
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, b['lower'], b['upper']),
                cv2.inRange(hsv, b['lower2'], b['upper2'])
            )
        else:
            mask = cv2.inRange(hsv, b['lower'], b['upper'])

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            rect = cv2.minAreaRect(contours[0])
            (rect_x, rect_y), (rect_w, rect_h), rect_a = rect
            if rect_w > rect_h:
                rect_w, rect_h = rect_h, rect_w
            self.last_rect_h = rect_h
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv_img = cv2.drawContours(cv_img, [box], 0, (0, 0, 0), 1)
            centroid = (int(rect_x), int(rect_y))
            cv2.circle(cv_img, centroid, 3, (0, 0, 0), -1)
            cv2.putText(cv_img, f"({rect_h:.1f}", (centroid[0] + 5, centroid[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        else:
            self.last_rect_h = None

        cv2.putText(cv_img, f"mode: {mode}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        self._image_pub.publish(self._bridge.cv2_to_imgmsg(cv_img))


def main(args=None):
    global mode, distance

    rclpy.init(args=args)
    node = DistanceCalibrationNode()
    settings = termios.tcgetattr(sys.stdin)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    csv_file = _csv_path()
    _write_csv(csv_file)
    print(f'CSV ready: {csv_file}')
    print(HELP)
    print(f'mode={mode}  distance={distance:.1f}')

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key == '\x03':
                break
            elif key == 'm':
                mode = {'red': 'green', 'green': 'blue', 'blue': 'red'}[mode]
                print(f'mode={mode}  distance={distance:.1f}')
            elif key == 'r':
                distance = 0.3
                print(f'distance reset -> {distance:.1f}  mode={mode}')
            elif key == ' ':
                h = node.last_rect_h
                h_str = f'{h:.1f}' if h is not None else ''
                _data_rows.append([mode, f'{distance:.1f}', h_str])
                _write_csv(csv_file)
                print(f'Logged: color={mode}  distance={distance:.1f}  rect_h={h_str}')
                distance = round(distance + 0.2, 1)
    finally:
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        base, ext = os.path.splitext(csv_file)
        ts_path = f'{base}_{ts}{ext}'
        _write_csv(ts_path)
        print(f'Saved timestamped copy: {ts_path}')
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
