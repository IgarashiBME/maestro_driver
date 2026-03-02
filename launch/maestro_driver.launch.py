"""Launch file for maestro_driver_node with configurable parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for the Pololu Maestro controller',
        ),
        DeclareLaunchArgument(
            'ch1_channel',
            default_value='0',
            description='Maestro channel number for ch1',
        ),
        DeclareLaunchArgument(
            'ch2_channel',
            default_value='1',
            description='Maestro channel number for ch2',
        ),
        DeclareLaunchArgument(
            'failsafe_timeout',
            default_value='0.5',
            description='Timeout in seconds before failsafe activates',
        ),
        DeclareLaunchArgument(
            'failsafe_pwm',
            default_value='1500',
            description='PWM value sent on failsafe (microseconds)',
        ),
        DeclareLaunchArgument(
            'pwm_min',
            default_value='1000',
            description='Minimum allowed PWM value (microseconds)',
        ),
        DeclareLaunchArgument(
            'pwm_max',
            default_value='2000',
            description='Maximum allowed PWM value (microseconds)',
        ),
        Node(
            package='maestro_driver',
            executable='maestro_driver_node',
            name='maestro_driver_node',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'ch1_channel': LaunchConfiguration('ch1_channel'),
                'ch2_channel': LaunchConfiguration('ch2_channel'),
                'failsafe_timeout': LaunchConfiguration('failsafe_timeout'),
                'failsafe_pwm': LaunchConfiguration('failsafe_pwm'),
                'pwm_min': LaunchConfiguration('pwm_min'),
                'pwm_max': LaunchConfiguration('pwm_max'),
            }],
            output='screen',
        ),
    ])
