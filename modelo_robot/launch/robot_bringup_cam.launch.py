import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    pkg = get_package_share_directory('modelo_robot')

    cam_ip_arg = DeclareLaunchArgument(
        'cam_ip',
        default_value='172.20.10.5',
        description='IP de la ESP32-CAM')

    cam_ip = LaunchConfiguration('cam_ip')

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'gazebo.launch.py')))

    # 2. micro-ROS agent nativo (desde ~/uros_ws)
    agent = ExecuteProcess(
        cmd=['bash', '-c',
             'source ~/uros_ws/install/setup.bash && '
             'ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4'],
        output='screen')

    # 3. Xbox → /joy
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }])

    # 4. joy_controller → /cmd_vel
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
            'cam_ip':              '172.20.10.5',
        }],
        output='screen')

    # 5. IMU broadcaster
    imu_broadcaster = Node(
        package='modelo_robot',
        executable='tf_broadcaster_imu',
        output='screen')

    # 6. ESP32-CAM → /camera/image_raw
    cam_node = Node(
        package='modelo_robot',
        executable='mjpeg_to_ros',
        name='mjpeg_to_ros',
        parameters=[{
            'stream_url': ['http://', cam_ip, ':81/stream'],
            'frame_id':   'camera_link',
        }],
        output='screen')

    return LaunchDescription([
        cam_ip_arg,
        gazebo,
        agent,
        joy_node,
        TimerAction(period=2.0, actions=[joy_controller]),
        TimerAction(period=3.0, actions=[imu_broadcaster]),
        TimerAction(period=4.0, actions=[cam_node]),
    ])
