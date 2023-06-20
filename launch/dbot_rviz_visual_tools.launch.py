import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import add_debuggable_node, DeclareBooleanLaunchArg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Initialize moveit_config
    moveit_config = MoveItConfigsBuilder("dbot_world", package_name="dbot_moveit_config").to_moveit_configs()

    # Declare arguments and nodes
    declared_arguments = []
    nodes = []

    # Static Transform Broadcaster
    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    if virtual_joints_launch.exists():
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Robot State Publisher
    # Given the published joint states, publish tf for the robot links and the robot description
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {
                "publish_frequency": 15.0,
            }
        ],
    )

    # MoveGroup
    move_group_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
    )

    # Rviz
    # Run Rviz and load the default config to see the state of the move_group node
    my_robot_description_path = get_package_share_path('dbot')
    rviz_config = my_robot_description_path / 'config/dbot_rviz_visual_tools.rviz'
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
        ),
        launch_arguments={
            "rviz_config":str(rviz_config),
        }.items(),
    )

    # Ros2 Control
    # Fake hardware driver
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    dbot_controller_spawner =   Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dbot_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # # Add nodes
    nodes.append(ros2_control_node)
    nodes.append(rsp_node)
    nodes.append(move_group_launch)
    nodes.append(rviz_launch)
    nodes.append(joint_state_broadcaster_spawner)
    nodes.append(dbot_controller_spawner)
    
    return LaunchDescription(declared_arguments + nodes)