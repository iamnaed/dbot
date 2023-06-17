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
    moveit_config = MoveItConfigsBuilder("dbot", package_name="dbot_moveit_config").to_moveit_configs()

    # Declare arguments and nodes
    declared_arguments = []
    nodes = []

    # Arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'dbot_urdf', 
            default_value=str(get_package_share_path('dbot') / 'urdf/dbot_gazebo.urdf.xacro'), 
            description='Absolute path to robot urdf file'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time", 
            default_value="True"
        )
    )

    # Initialize Arguments
    dbot_urdf = LaunchConfiguration("dbot_urdf")
    use_sim_time = LaunchConfiguration("use_sim_time")    

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
    robot_description = ParameterValue(
        Command(['xacro ', dbot_urdf]), 
        value_type=str
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            {
                "robot_description" : robot_description,
                "publish_frequency": 15.0,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # MoveGroup
    move_group_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
            launch_arguments={
                "use_sim_time":use_sim_time,
            }.items(),
    )

    # Rviz
    # Run Rviz and load the default config to see the state of the move_group node
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
        ),
    )

    # Gazebo
    # Include the Gazebo launch file, provided by the gazebo_ros package
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        ),
    )
    
    gazebo_spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description','-entity', 'dbot_gz'],
        output='screen',
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    dbot_controller_spawner =   Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dbot_controller"],
        output="screen",
    )
    
    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_dbot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[dbot_controller_spawner],
        )
    )

    # Delay start of move_group_launch after `joint_state_broadcaster`
    delay_move_group_launch_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[move_group_launch],
        )
    )

    # Delay rviz start after `gazebo_launch`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_launch],
        )
    )

    # Add nodes
    nodes.append(robot_state_publisher_node)
    nodes.append(gazebo_launch)
    nodes.append(gazebo_spawn_entity)
    nodes.append(joint_state_broadcaster_spawner)
    nodes.append(delay_dbot_controller_spawner_after_joint_state_broadcaster_spawner)
    nodes.append(delay_move_group_launch_after_joint_state_broadcaster_spawner)
    nodes.append(delay_rviz_after_joint_state_broadcaster_spawner)
    
    return LaunchDescription(declared_arguments + nodes)