from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('modelo_robot')

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'gazebo.launch.py')))

    # 2. micro-ROS agent
    agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
             'udp4', '--port', '8888'],
        output='screen')

    # 3. Xbox → /cmd_vel
    joy = Node(package='joy', executable='joy_node',
               parameters=[{'dev': '/dev/input/js0'}])

    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        parameters=[{'joy_config': 'xbox'}],
        remappings=[('/cmd_vel', '/cmd_vel')])

    # 4. IMU broadcaster
    imu_broadcaster = Node(
        package='modelo_robot',
        executable='tf_broadcaster_imu',
        output='screen')

    return LaunchDescription([
        gazebo, agent, joy,
        TimerAction(period=2.0, actions=[teleop]),
        TimerAction(period=3.0, actions=[imu_broadcaster]),
    ])
