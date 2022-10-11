# Importing Libraries
import serial

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor



class GripperController(Node):

    def __init__(self):
        super().__init__('gripper_controller')
        self.arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=.1)
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()

        self.motor_sub = self.create_subscription(
            JointState,
            't42_motor_control',
            self.motor_callback,
            10,
            callback_group=self.group2)
        
        self.states_publisher = self.create_publisher(JointState, 't42_motor_states', 10)

        self.timer = self.create_timer(.1, self.timer_callback, callback_group=self.group1)
        self.requested = {"right": 0, "left":0 }
        

    def timer_callback(self):
        # for el in self.arduino.readlines():
        #     try:
        #         self.get_logger().info(el.decode('utf-8'))
        #     except:
        #         print(el)
        tmp = self.arduino.read()
        try:
            txt = tmp.decode("utf-8")
        except:
            print(tmp)
            return
        while tmp and tmp != '\n':
            tmp =  self.arduino.read()
            try:
                txt += tmp.decode("utf-8")
            except:
                print(tmp)
                return
        if txt:
            self.parse_response(txt)
        self.arduino.write(f"Right:{self.requested['right']},Left:{self.requested['left']}\n".encode('utf-8'))

    def convert_to_degrees(self, val):
        return val*360/4095

    def convert_from_degrees(self, val):
        return val*4095/360

    def parse_response(self, response):
        print(response)
        try:
            response = response.split('/')
            type = response[0]
            motors = response[1].split(',')
            right = motors[0].split(':')[-1]
            left = motors[1].split(':')[-1]
        except:
            print(f"Failed to handle {response}")
            return

        msg = JointState()
        msg.name = ["right", "left"]
        if type == "Pose":
            msg.position = [self.convert_to_degrees(int(right)), self.convert_to_degrees(int(left))] 
        elif type == "Load":
            msg.effort = [float(right), float(left)]
        self.states_publisher.publish(msg)
        
    def motor_callback(self, msg:JointState):
        for idx, key in enumerate(msg.name):
            self.requested[key] = self.convert_from_degrees(msg.position[idx])
   


def main(args=None):
    rclpy.init(args=args)

    gripper_controller = GripperController()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(gripper_controller)

    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    executor.shutdown()
    gripper_controller.destroy_node()


if __name__ == '__main__':
    main()