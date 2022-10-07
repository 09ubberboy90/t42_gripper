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

        self.right_publisher = self.create_publisher(Float32, 'right_motor_control', 10)
        self.left_publisher = self.create_publisher(Float32, 'left_motor_control', 10)

        self.timer = self.create_timer(.25, self.timer_callback)
        self.left = 0.0
        self.right = 0.0
        self.counter = 0

        self.right_vals = np.zeros((5), np.float32)
        self.left_vals = np.zeros((5), np.float32)

    def timer_callback(self):
        # for el in self.arduino.readlines():
        #     try:
        #         self.get_logger().info(el.decode('utf-8'))
        #     except:
        #         print(el)

        
        msg = Float32()
        msg.data = float(self.right)
        self.right_publisher.publish(msg)
        msg.data = float(self.left)
        self.left_publisher.publish(msg)
        if self.counter < 200:    
            self.right += 1
            self.left += 1
        if 200 < self.counter < 400:
            self.right -= 1
            self.left -= 1
        if self.counter > 600:
            self.counter = 0
        self.counter += 1
        



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