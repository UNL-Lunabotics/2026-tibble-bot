from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    control_pkg = FindPackageShare("control")
    bringup_pkg = FindPackageShare("bringup")

    # --- Nodes ---
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'device_id': 0}]
    )

    state_manager_node = Node(
        package='control',
        executable='state_manager_node',
        name='state_manager_node',
        output='screen'
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[PathSubstitution(control_pkg) / "config" / "joystick.yaml"]
    )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", PathSubstitution(bringup_pkg) / "config" / "teleop.rviz"],
    #     condition=IfCondition(LaunchConfiguration("gui")),
    # )
    
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765
        }]
    )
    
    foxglove_web_app = ExecuteProcess(
        cmd=['xdg-open', 'http://localhost:8080'],
        shell=True
    )


    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        joy_node,
        state_manager_node,
        teleop_node,
        # rviz_node
        foxglove_bridge,
        foxglove_web_app,
    ])