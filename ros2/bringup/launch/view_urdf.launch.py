from launch import LaunchDescription
from launch.substitutions import Command, PathSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


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
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description_str
                    },
                ],
            ),
            # RViz2 node
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    PathSubstitution(FindPackageShare("bringup"))
                    / "config"
                    / "view_urdf.rviz",
                ],
            ),
        ]
    )