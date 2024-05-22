from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topic_tools',
            executable='relay',
            name='ublox_nav_fix_topic_relay',
            output='screen',
            arguments=['ublox_gps_node/fix', 'sensing/gnss/ublox_gps_node/fix']
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='ublox_twist_topic_relay',
            output='screen',
            arguments=['ublox_gps_node/fix_velocity', 'sensing/gnss/ublox_gps_node/fix_velocity']
        ),
    ])