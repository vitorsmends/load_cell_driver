from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare('load_cell_driver'),
        'config',
        'config.yaml'
    ])

    return LaunchDescription([
        Node(
            package='load_cell_driver',
            executable='force_sensor',
            name='force_sensor_node',
            output='screen',
            parameters=[config_path]
        )
    ])
