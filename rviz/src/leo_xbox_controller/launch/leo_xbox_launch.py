from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joy node - reads controller hardware
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),
        
        # Custom controller node - handles teleop and return home
        Node(
            package='leo_xbox_controller',
            executable='leo_xbox_controller.py',
            name='leo_xbox_controller',
            output='screen',
        )
    ])
