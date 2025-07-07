from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare first
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    # Config path
    config_file = PathJoinSubstitution([
        FindPackageShare('faster_lio'),
        'config',
        'avia.yaml'
    ])

    # Node 1
    laser_mapping_node = Node(
        package='faster_lio',
        executable='run_mapping_online',
        name='laserMapping',
        output='screen',
        parameters=[config_file]
    )

    # Node 2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', '/home/arthurycliu/repo/faster-lio/src/faster_lio/rviz_cfg/loam_livox.rviz'],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Make sure rviz_arg is included BEFORE it's used
    return LaunchDescription([
        rviz_arg,
        laser_mapping_node,
        rviz_node
    ])
