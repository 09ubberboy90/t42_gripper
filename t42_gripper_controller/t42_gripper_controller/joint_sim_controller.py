# Importing Libraries
import serial
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import numpy as np

class GripperController(Node):

    def __init__(self):
        super().__init__('gripper_controller')
        self.sim_pub = self.create_publisher(JointState, 'gripper_joint_states', 10)
        
        self.glove_pub = self.create_publisher(Float64MultiArray, '/senseglove/rh/joint_position_controller/commands', 10)
        self.glove_sub = self.create_subscription(JointState, '/senseglove/rh/joint_states',self.senseglove_callback, 10)

        self.motor_pub = self.create_publisher(JointState, 't42_motor_control', 10)
        self.motor_sub = self.create_subscription(JointState, 't42_motor_states', self.motor_callback, 10)

        self.thumb_history = np.zeros(10)
        self.index_history = np.zeros(10)
        self.thumb_position = np.zeros(5)
        self.index_position = np.zeros(5)

    def motor_callback(self, incoming:JointState):
        outgoing = Float64MultiArray()
        idx_remap = {}
        if len(incoming.name) == 0:
            return
        for idx, key in enumerate(incoming.name):
            idx_remap[key] = idx
        try:
            self.thumb_history = np.roll(self.thumb_history,-1)
            self.thumb_history[-1] = self.adjust_feedback(incoming.effort[idx_remap["right"]])
            self.index_history = np.roll(self.index_history,-1)
            self.index_history[-1] = self.adjust_feedback(incoming.effort[idx_remap["left"]])

            print("thumb",self.thumb_history)
            print("index", self.index_history)
            outgoing.data = [
                        # np.mean(self.thumb_history), 
                        # np.mean(self.index_history),
                        0.0,0.0,0.0,0.0,0.0,
                        np.median(self.thumb_history), 
                        np.median(self.index_history),
                        0.0,0.0,0.0] # no vibration
        except:
            return
        self.glove_pub.publish(outgoing)

    def senseglove_callback(self, incoming:JointState):
        outgoing = JointState()
        outgoing.name = ["right", "left"]
        joint_dict = {}
        for idx, key in enumerate(incoming.name):
            joint_dict[key] = incoming.position[idx]
        thumb = -joint_dict["thumb_mcp"] +  joint_dict["thumb_pip"]  # 1.5 - 3.3
        index = -joint_dict["index_pip"] +  joint_dict["index_dip"]  #0.5 3.3
        self.thumb_position = np.roll(self.thumb_position,-1)
        self.thumb_position[-1] = self.thumb_degree(thumb)
        self.index_position = np.roll(self.index_position,-1)
        self.index_position[-1] = self.index_degree(index)


        outgoing.position = [np.mean(self.thumb_position), np.mean(self.index_position)] 
        self.motor_pub.publish(outgoing)
        
    def adjust_feedback(self, val):
        val = max(min(val,512),0) #clamp to 50% max load
        #NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
        return float((abs(val)%512) * 100 / 512)
    
    def index_degree(self, val):
        #NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
        return min(max(((val - 1.5) * 250) / (3.3 - 1.5), 0.0),250.0) ## Max 250 degree for safety
    
    def thumb_degree(self, val):
        #NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
        return min(max(((val - 1.5) * 250) / (3.3 - 1.5), 0.0),250.0) ## Max 250 degree for safety




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