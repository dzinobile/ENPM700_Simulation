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
  Ctrl+C: quit
"""

mode = 'red'


def get_key(settings):
	tty.setraw(sys.stdin.fileno())
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key





class Foxglove(Node):
	def __init__(self):
		super().__init__('foxglove_node')
		self._image_pub = self.create_publisher(Image, 'foxglove_image', 1)
		self._bridge = CvBridge()

		self.create_timer(0.1, self._timer_callback)

	def _timer_callback(self):
		height = 480
		width = 640
		channels = 3
	
		cv_img = np.zeros((height, width, channels), dtype=np.uint8)

		if mode == 'red':
			cv_img[:, :, 2] = 255
		elif mode == 'blue':
			cv_img[:, :, 0] = 255
		else:
			cv_img[:, :, 1] = 255

		self._image_pub.publish(self._bridge.cv2_to_imgmsg(cv_img))

	def destroy_node(self):
		super().destroy_node()


def main(args=None):
	global mode
	rclpy.init(args=args)
	node = Foxglove()
	settings = termios.tcgetattr(sys.stdin)

	spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
	spin_thread.start()


	try:
		while rclpy.ok():
			key = get_key(settings)

			if key == '\x03':
				break
			elif key == 'm':
				mode = {'red': 'green', 'green': 'blue', 'blue': 'red'}[mode]
	finally:
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
