import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, Int16
import numpy as np

class JointController(Node):

    def __init__(self):
        super().__init__('baxter_joint_controller')
        self.publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.motor_pub = self.create_publisher(JointState, 't42_motor_control', 10)

        self.rmotor_sub = self.create_subscription(Int16, 'right_motor', self.right_motor_callback, 10)
        self.lmotor_sub = self.create_subscription(Int16, 'left_motor', self.left_motor_callback, 10)

        self.gripper_names = {
            "left": "swivel_1_to_finger_1_1",
            "right": "swivel_2_to_finger_2_1",
        }

    def right_motor_callback(self, msg:Int16):
        outgoing = JointState()
        outgoing.name.append("right")
        outgoing.position.append(msg.data)
        self.motor_pub.publish(outgoing)
    
    def left_motor_callback(self, msg:Int16):
        outgoing = JointState()
        outgoing.name.append("left")
        outgoing.position.append(msg.data)
        self.motor_pub.publish(outgoing)


def main(args=None):
    rclpy.init(args=args)

    node = JointController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
