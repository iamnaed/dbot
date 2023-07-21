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
    """
    Launches dbot with moveit

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * ros2_control_node + controller spawners
    """
    moveit_config = MoveItConfigsBuilder("dbot_world", package_name="dbot_moveit_config").to_moveit_configs()
    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg("use_rviz", default_value=True)
    )
    # Given the published joint states, publish tf for the robot links
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="60.0"))

    # Static Transform Broadcaster
    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Robot State Publisher
    # Given the published joint states, publish tf for the robot links and the robot description
    dbot_share_path = get_package_share_path('dbot')
    dbot_urdf_path = dbot_share_path / 'urdf/dbot_can_hardware.urdf.xacro'
    ld.add_action(
        DeclareLaunchArgument(name='dbot_urdf', default_value=str(dbot_urdf_path),
                                      description='Absolute path to robot urdf file')
    )
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('dbot_urdf')]), value_type=str
    )
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            {
                "robot_description" : robot_description,
                "publish_frequency": 15.0,
            }
        ],
    )
    ld.add_action(rsp_node)

    # MoveGroup
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Rviz
    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            ),
        )
    )

    # Ros2 Control
    # CAN hardware driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {
                    "robot_description" : robot_description,
                },
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["dbot_arm_controller"],
            output="screen",
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        )
    )

    return ld