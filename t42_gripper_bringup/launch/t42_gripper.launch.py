import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression,Command,PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("t42_gripper_description"), "urdf", "t42_hand.xacro"]),
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,            
            " ",
            "name:=",
            "t42_gripper",            
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description,],
        # {"frame_prefix": "t42/"}],
        #namespace="t42"

    )
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("t42_gripper_bringup"), "config", "controllers.yaml"]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, initial_joint_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        condition=IfCondition(use_fake_hardware),
        #namespace="t42"
    )
    load_controllers = []
    for controller in [
        "t42_gripper_controller",
        "t42_joint_state_broadcaster",
    ]:
        load_controllers += [
            Node(
            package="controller_manager",
            executable="spawner",
            arguments=[f"{controller}",
                       "--controller-manager", "controller_manager"],
        ),
        ]
    for controller in [
        "t42_right_trajectory_controller",
        "t42_left_trajectory_controller",
    ]:
        load_controllers += [
            Node(
            package="controller_manager",
            executable="spawner",
            arguments=[f"{controller}",
                       "--controller-manager", "controller_manager", "--stopped"],
        ),
        ]
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="True",
            description="Start robot with fake hardware mirroring command to its states.",
        ),
        robot_state_publisher,
        control_node
    ] + load_controllers

    )