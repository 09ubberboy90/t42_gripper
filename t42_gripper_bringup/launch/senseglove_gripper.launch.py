import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import  PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.actions import ExecuteProcess



def generate_launch_description():

    ld = LaunchDescription()

    controller = Node(package="t42_gripper_controller",
                         executable="gripper_controller",)
    joint_sim_controller = Node(package="t42_gripper_controller",
                         executable="joint_sim_controller",)

    senseglove = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("senseglove_launch"), 'launch', 'senseglove_hardware_demo.launch.py'),
        ),)
    sensecom = ExecuteProcess(
    cmd=["/home/ubb/Documents/Baxter_isaac/ROS2/src/senseglove_ros2_ws/SenseCom/Linux/SenseCom.x86_64"],
    shell=True
    )

    ld.add_action(controller)
    ld.add_action(sensecom)
    ld.add_action(TimerAction(
            period=5.,
            actions=[
                senseglove,
                joint_sim_controller,
            ]))


    return ld
