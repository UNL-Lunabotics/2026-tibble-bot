from launch import LaunchDescription
from launch.substitutions import Command, PathSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    robot_description = Command(
        [
            "xacro",
            " ",
            PathSubstitution(FindPackageShare("description"))
            / "urdf"
            / "tibble.urdf.xacro",
        ]
    )
    
    # realsense_pkg = FindPackageShare("realsense2_camera")
    
    # camera_front = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([realsense_pkg, '/launch/rs_launch.py']),
    #     launch_arguments={
    #         'camera_name': 'camera_front',
    #         'serial_no': "'033322071026'",
    #         'enable_depth': 'true',
    #         'enable_color': 'true',
    #     }.items()
    # )

    # Python based nodes need more strict param stuff it's annoying
    robot_description_str = ParameterValue(robot_description, value_type=str)

    return LaunchDescription(
        [
            # robot state publisher with robot_description from xacro
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description_str
                    }
                ],
            ),
            # joint state publisher gui for joint manipulation
            # Node(
            #     package="joint_state_publisher_gui",
            #     executable="joint_state_publisher_gui",
            #     output="screen",
            #     parameters=[
            #         {
            #             "robot_description": robot_description_str
            #         },
            #     ],
            # ),
            # RViz2 node
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     output="screen",
            #     arguments=[
            #         "-d",
            #         PathSubstitution(FindPackageShare("bringup"))
            #         / "config"
            #         / "view_urdf.rviz",
            #     ],
            # ),
            # Foxglove Node
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
            ),
            # camera_front,
        ]
    )

# To run it locally with webapp run RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 launch bringup view_urdf.launch.py
# and go to https://app.foxglove.dev/unl-lunabotics/view?layoutId=lay_0eLO4KY3A3i9Zriw&ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A8765