import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/ovo/robotics-studio-1/src/my_map.yaml',
                'use_sim_time': True
            }],
        ),
    ])
