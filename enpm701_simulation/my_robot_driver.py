import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_back_motor = self.__robot.getDevice('left back wheel motor')
        self.__right_back_motor = self.__robot.getDevice('right back wheel motor')
        self.__left_front_motor = self.__robot.getDevice('left front wheel motor')
        self.__right_front_motor = self.__robot.getDevice('right front wheel motor')
        self.__left_gripper_slider_motor = self.__robot.getDevice('left gripper slider motor')
        self.__right_gripper_slider_motor = self.__robot.getDevice('right gripper slider motor')
        self.__left_gripper_lift_motor = self.__robot.getDevice('left gripper lift motor')
        self.__right_gripper_lift_motor = self.__robot.getDevice('right gripper lift motor')
        self.__left_front_sensor = self.__robot.getDevice('left front wheel sensor')
        self.__right_back_sensor = self.__robot.getDevice('right back wheel sensor')
        timestep = int(self.__robot.getBasicTimeStep())
        self.__left_front_sensor.enable(timestep)
        self.__right_back_sensor.enable(timestep)

        self.__left_back_motor.setPosition(float('inf'))
        self.__left_back_motor.setVelocity(0)
        self.__left_front_motor.setPosition(float('inf'))
        self.__left_front_motor.setVelocity(0)

        self.__right_back_motor.setPosition(float('inf'))
        self.__right_back_motor.setVelocity(0)
        self.__right_front_motor.setPosition(float('inf'))
        self.__right_front_motor.setVelocity(0)

        self.__left_gripper_slider_motor.setVelocity(0.5)
        self.__right_gripper_slider_motor.setVelocity(0.5)
        self.__left_gripper_lift_motor.setVelocity(0.5)
        self.__right_gripper_lift_motor.setVelocity(0.5)

        self.__target_twist = Twist()
        self.__gripper_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(Twist, 'gripper_vel', self.__gripper_vel_callback, 1)
        self.__left_wheel_pos_pub = self.__node.create_publisher(Float64, 'wheel_position/left', 1)
        self.__right_wheel_pos_pub = self.__node.create_publisher(Float64, 'wheel_position/right', 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist
    
    def __gripper_vel_callback(self, twist):
        self.__gripper_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z
        gripper_position = self.__gripper_twist.linear.x
        gripper_lift = self.__gripper_twist.linear.y

        command_motor_left_back = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right_back = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_left_front = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right_front = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_gripper_left = (-gripper_position)
        command_gripper_right = (gripper_position)
        command_gripper_lift = (gripper_lift)

        self.__left_back_motor.setVelocity(command_motor_left_back)
        self.__right_back_motor.setVelocity(command_motor_right_back)
        self.__left_front_motor.setVelocity(command_motor_left_front)
        self.__right_front_motor.setVelocity(command_motor_right_front)
        self.__left_gripper_slider_motor.setPosition(command_gripper_left)
        self.__right_gripper_slider_motor.setPosition(command_gripper_right)
        self.__left_gripper_lift_motor.setPosition(command_gripper_lift)
        self.__right_gripper_lift_motor.setPosition(command_gripper_lift)

        left_pos_msg = Float64()
        left_pos_msg.data = self.__left_front_sensor.getValue()
        self.__left_wheel_pos_pub.publish(left_pos_msg)

        right_pos_msg = Float64()
        right_pos_msg.data = self.__right_back_sensor.getValue()
        self.__right_wheel_pos_pub.publish(right_pos_msg)