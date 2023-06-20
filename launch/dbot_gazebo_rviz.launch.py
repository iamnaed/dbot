import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """
    Launches dbot with moveit

    Includes
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * ros2_control_node + controller spawners
    """
    moveit_config = MoveItConfigsBuilder("dbot_world", package_name="dbot_moveit_config").to_moveit_configs()
    ld = LaunchDescription()

    # Robot State Publisher
    # Given the published joint states, publish tf for the robot links and the robot description
    dbot_share_path = get_package_share_path('dbot')
    dbot_urdf_path = dbot_share_path / 'urdf/dbot_gazebo.urdf.xacro'
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
                "use_sim_time": True,
            }
        ],
    )
    ld.add_action(rsp_node)

    # Include the Gazebo launch file, provided by the gazebo_ros package
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'dbot_gz'],
                    output='screen')
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)

    # Rviz
    rviz_config_path = dbot_share_path / 'config/robot.rviz'
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(rviz_config_path)],
            parameters=[{'use_sim_time': True}]
        )
    )

    # Ros 2 Control
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