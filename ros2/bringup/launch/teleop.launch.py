from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    control_pkg = FindPackageShare("control")
    description_pkg = FindPackageShare("description")
    bringup_pkg = FindPackageShare("bringup")

    robot_description_content = Command(
        [
            "xacro ",
            PathSubstitution(description_pkg) / "urdf" / "tibble.urdf.xacro",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

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
        parameters=[PathSubstitution(control_pkg) / "config" / "joystick.yaml"],
        remappings=[('/cmd_vel', '/tibble_controller/cmd_vel')]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathSubstitution(control_pkg) / "config" / "controllers.yaml"
        ],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    tibble_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tibble_controller", "--controller-manager", "/controller_manager"],
    )

    delay_tibble_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[tibble_controller_spawner],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathSubstitution(bringup_pkg) / "config" / "teleop.rviz"],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        joy_node,
        teleop_node,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_tibble_controller_spawner,
        rviz_node
    ])