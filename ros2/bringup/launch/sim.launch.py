import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
  
  control_pkg = FindPackageShare("control")
  description_pkg = FindPackageShare("description")
  bringup_pkg = FindPackageShare("bringup")
  slam_toolbox_pkg = FindPackageShare("slam_toolbox")
  nav2_bringup_pkg = FindPackageShare("nav2_bringup")
  ros_gz_sim_pkg = FindPackageShare("ros_gz_sim")
    
  bridge_params = os.path.join(
      bringup_pkg.find("bringup"),
      "config",
      "gz_bridge.yaml"
  )
  
  world_path = os.path.join(
    description_pkg.find("description"),
    "worlds",
    "artemis_arena.sdf"
  )
  
  gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          [ros_gz_sim_pkg, "/launch/gz_sim.launch.py"]
      ),
      launch_arguments={"gz_args": "-r " + world_path}.items(),
  )
  
  robot_description_content = ParameterValue(
      Command([
          "xacro ",
          PathSubstitution(description_pkg) / "urdf" / "tibble.urdf.xacro",
          " use_sim:=true"
      ]),
      value_type=str
  )
  robot_description = {"robot_description": robot_description_content}
  
  joy_node = Node(
    package = "joy",
    executable = "joy_node",
    name = "joy_node",
    parameters = [{"device_id": 0}]
  )
  
  teleop_node = Node(
    package = "teleop_twist_joy",
    executable = "teleop_node",
    name = "teleop_twist_joy_node",
    parameters = [PathSubstitution(control_pkg) / "config" / "joystick.yaml"],
    remappings = [("/cmd_vel", "/tibble_controller/cmd_vel")]
  )
  
  control_node = Node(
    package = "controller_manager",
    executable = "ros2_control_node",
    parameters = [
      robot_description,
      PathSubstitution(control_pkg) / "config" / "tibble_controller.yaml"
    ],
    output = "both"
  )
  
  robot_state_pub_node = Node(
    package = "robot_state_publisher",
    executable = "robot_state_publisher",
    output = "both",
    parameters = [robot_description]
  )
  
  bridge = Node(
    package = "ros_gz_bridge",
    executable = "parameter_bridge",
    name = "gazebo",
    parameters = [
      {"config_file": bridge_params},
      {"publish_rate": 400.0},
      {"qos_overrides./tf_static.publisher.durability": "transient_local"}
    ],
    output="screen"
  )
  
  spawn_tibble = Node(
    package = "ros_gz_sim",
    executable = "create",
    arguments=[
      "-topic", "robot_description",
      "-name", "terrence",
      "-z", "0.5",
    ],
    output = "both"
  )
  
  joint_state_broadcaster_spawner = Node(
    package = "controller_manager",
    executable = "spawner",
    arguments = ["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
  )
  
  tibble_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["tibble_controller", "--controller-manager", "/controller_manager"]
  )
  
  delay_joint_state_broadcaster_spawner = RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=spawn_tibble,
      on_exit=[joint_state_broadcaster_spawner]
    )
  )
  
  delay_tibble_controller_spawner = RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=joint_state_broadcaster_spawner,
      on_exit=[tibble_controller_spawner]
    )
  )
    
  rviz_node = Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      output="log",
      arguments=["-d", PathSubstitution(bringup_pkg) / "config" / "teleop.rviz"],
      condition=IfCondition(LaunchConfiguration("gui"))
  )
  
  foxglove_bridge = Node(
    package="foxglove_bridge",
    executable="foxglove_bridge",
    name="foxglove_bridge",
  )
  
  slam_toolbox = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [PathSubstitution(slam_toolbox_pkg), "/launch", "/online_async_launch.py"]
    ),
    launch_arguments={"use_sim_time": "true"}.items(),
  )
  
  nav2_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [PathSubstitution(nav2_bringup_pkg), "/launch/navigation_launch.py"]
    ),
    launch_arguments={
      "use_sim_time": "true",
      "params_file" : [PathSubstitution(bringup_pkg), "/config/nav2_params.yaml"]
      }.items(),
  )
  
  ekf_node = Node(
    package="robot_localization",
    executable="ekf_node",
    name="ekf_filter_node",
    output="both",
    parameters=[
      PathSubstitution(bringup_pkg) 
      / "config" 
      / "ekf_params.yaml",
      {"use_sim_time": True}
    ]
  )
  
  return LaunchDescription([
    DeclareLaunchArgument("gui", default_value="true", description="Whether to launch RViz"),
    gazebo,
    joy_node,
    teleop_node,
    control_node,
    robot_state_pub_node,
    bridge,
    spawn_tibble,
    delay_joint_state_broadcaster_spawner,
    delay_tibble_controller_spawner,
    rviz_node,
    foxglove_bridge,
    slam_toolbox,
    nav2_bringup,
    ekf_node
  ])