"""
ps_teleop.launch.py
───────────────────
Forza Horizon-style PS4/PS5 teleop for the FS car simulator.
Run on a second terminal while the simulator is running.

Usage:
  ros2 launch fs_vehicle ps_teleop.launch.py
  ros2 launch fs_vehicle ps_teleop.launch.py joy_dev:=/dev/input/js1
  ros2 launch fs_vehicle ps_teleop.launch.py max_speed:=20.0

Controls:
  R2              — throttle (analog)
  L2              — brake when moving / reverse when stopped (analog)
  Left stick L/R  — steering (speed-sensitive)
  Cross / A       — handbrake
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument('joy_dev',     default_value='/dev/input/js0',
            description='Joystick device path'),
        DeclareLaunchArgument('max_speed',   default_value='15.0',
            description='Max forward speed at full R2 press (m/s)'),
        DeclareLaunchArgument('max_reverse', default_value='5.0',
            description='Max reverse speed at full L2 reverse (m/s)'),

        # joy_node: reads raw joystick events → sensor_msgs/Joy
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            parameters=[{
                'dev':             LaunchConfiguration('joy_dev'),
                'deadzone':        0.05,  # ignore small stick/trigger drift
                'autorepeat_rate': 0.0,   # only publish on input change
            }],
        ),

        # ps_teleop: converts Joy → Twist on /cmd_vel
        Node(
            package='fs_vehicle',
            executable='ps_teleop',
            name='ps_teleop',
            output='screen',
            parameters=[{
                'max_speed':   LaunchConfiguration('max_speed'),
                'max_reverse': LaunchConfiguration('max_reverse'),
            }],
        ),
    ])
