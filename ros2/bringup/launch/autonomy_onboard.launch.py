from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    control_pkg = FindPackageShare("control")
    description_pkg = FindPackageShare("description")
    realsense_pkg = FindPackageShare("realsense2_camera")

    robot_description_content = ParameterValue(
        Command(
            [
                "xacro ",
                PathSubstitution(description_pkg) / "urdf" / "tibble.urdf.xacro",
                " use_sim:=false",
                " use_control:=true"
            ]
        ),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # --- Hardware & Control Nodes ---
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathSubstitution(control_pkg) / "config" / "tibble_controller.yaml"
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

    # --- Sensor Nodes ---
    camera_front = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense_pkg, '/launch/rs_launch.py']),
        launch_arguments={
            'camera_name': 'camera_front',
            'serial_no': '033322071026',
            'enable_depth': 'true',
            'enable_color': 'true',
        }.items()
    )

    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '1000000',
            'frame_id': 'laser'}
        ],
        output='screen'
    )
    
    # --- Navigation & Mapping ---
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathSubstitution(FindPackageShare("slam_toolbox")), "/launch/online_async_launch.py"]
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )
    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathSubstitution(FindPackageShare("nav2_bringup")), "/launch/navigation_launch.py"]
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': [PathSubstitution(FindPackageShare("bringup")), "/config/nav2_params.yaml"]
        }.items(),
    )

    # THE BOOT NODE (Auto-Localization via AprilTag)
    boot_node = Node(
        package='autonomy',
        executable='boot_node.py',
        name='tibble_boot_node',
        output='screen',
        parameters=[{
            'target_tag_frame': 'tag_36h11_id0', # TODO: change if other april tag type
            'global_map_frame': 'map',
            'robot_base_frame': 'base_link'
        }]
    )

    bt_executor_node = Node(
        package='autonomy',
        executable='autonomy_executor',
        name='autonomy_executor',
        output='screen',
        parameters=[{
            'tree_xml_file': 'main_tree.xml'
        }]
    )

    sequence_executor_after_boot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=boot_node,
            on_exit=[
                LogInfo(msg="[LAUNCH] Boot Node completed localization. Starting Behavior Tree Executor..."),
                bt_executor_node
            ]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_tibble_controller_spawner,
        camera_front,
        rplidar,
        slam_toolbox,
        nav2_bringup,
        boot_node,
        sequence_executor_after_boot
    ])