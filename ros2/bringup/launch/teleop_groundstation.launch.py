from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    control_pkg = FindPackageShare("control")

    # --- Nodes ---
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'device_id': 0}]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[PathSubstitution(control_pkg) / "config" / "joystick.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        joy_node,
        teleop_node,
    ])