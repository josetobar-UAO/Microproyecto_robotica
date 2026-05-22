import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
 
 
def generate_launch_description():
    pkg = get_package_share_directory('modelo_robot')
 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'gazebo.launch.py')))
 
    agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
             'udp4', '--port', '8888', '-v4'],
        output='screen')
 
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,        # deadzone minimo — lo maneja joy_controller
            'autorepeat_rate': 20.0,
        }])
 
    # Nodo propio en vez de teleop_twist_joy — control completo
    joy_controller = Node(
        package='modelo_robot',
        executable='joy_controller',
        name='joy_controller',
        parameters=[{
            'scale_linear':        0.3,
            'scale_linear_turbo':  0.5,
            'scale_angular':       1.5,
            'reverse_speed':       0.25,
            'deadzone':            0.25,
        }],
        output='screen')
 
    imu_broadcaster = Node(
        package='modelo_robot',
        executable='tf_broadcaster_imu',
        output='screen')
 
    return LaunchDescription([
        gazebo,
        agent,
        joy_node,
        TimerAction(period=2.0, actions=[joy_controller]),
        TimerAction(period=3.0, actions=[imu_broadcaster]),
    ])
 
