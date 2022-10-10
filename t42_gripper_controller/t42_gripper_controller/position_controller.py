# Importing Libraries
import serial
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
import numpy as np

class GripperController(Node):

    def __init__(self):
        super().__init__('gripper_controller')
        self.arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=.1)

        self.right_motor = self.create_subscription(
            Float32,
            'right_motor_control',
            self.right_callback,
            10)
        self.left_motor = self.create_subscription(
            Float32,
            'left_motor_control',
            self.left_callback,
            10)

        self.right_publisher = self.create_publisher(Float32, 'right_motor_states', 10)
        self.left_publisher = self.create_publisher(Float32, 'left_motor_states', 10)

        self.timer = self.create_timer(.1, self.timer_callback)
        self.left = 0.0
        self.right = 0.0

        self.right_vals = np.zeros((5), np.float32)
        self.left_vals = np.zeros((5), np.float32)

    def timer_callback(self):
        # for el in self.arduino.readlines():
        #     try:
        #         self.get_logger().info(el.decode('utf-8'))
        #     except:
        #         print(el)
        txt = ""
        tmp =  self.arduino.read()
        while tmp and tmp != '\n':
            tmp =  self.arduino.read()
            txt += tmp.decode("utf-8")
        if txt:
            self.parse_response(txt)
        self.arduino.write(f"Right:{self.right},Left:{self.left}\n".encode('utf-8'))

    def convert_to_degrees(self, val):
        return val*360/4095

    def convert_from_degrees(self, val):
        return val*4095/360

    def parse_response(self, response):
        motors = response.split(',')
        try:
            right = self.convert_to_degrees(int(motors[0].split(':')[-1]))
            left = self.convert_to_degrees(int(motors[1].split(':')[-1]))
        except:
            print(f"Failed to handle {response}")
            return
        self.right_vals = np.roll(self.right_vals, -1)
        self.left_vals = np.roll(self.left_vals,-1)
        self.right_vals[-1] = right
        self.left_vals[-1] = left
        right_mean = np.median(self.right_vals)
        left_mean = np.median(self.left_vals)
        msg = Float32()
        print(f"{right} {left}")
        msg.data = float(right_mean)
        self.right_publisher.publish(msg)
        msg.data = float(left_mean)
        self.left_publisher.publish(msg)
        
    def right_callback(self, msg:Float32):
        self.right = self.convert_from_degrees(msg.data)

    def left_callback(self, msg):
        self.left = self.convert_from_degrees(msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = GripperController()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()