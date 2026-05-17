import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'urdftest'
    urdf_file_name = 'urdf/tibbsassembly_v5point5.urdf'
    
    # Resolve the path to the URDF file
    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        urdf_file_name
    )

    # Read the URDF file content to pass it as a parameter
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 1. robot_state_publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 2. joint_state_publisher_gui
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        # Passed an empty dictionary for 'zeros' to mirror your empty <rosparam> tag
        parameters=[{'zeros': {}}] 
    )

    # 3. tf2_ros static_transform_publisher (map -> root)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_root',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'root'] 
    )

    # 4. rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        jsp_gui_node,
        static_tf_node,
        rviz_node
    ])