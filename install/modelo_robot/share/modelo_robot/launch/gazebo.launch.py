import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg = get_package_share_directory('modelo_robot')
    urdf_file = os.path.join(pkg, 'urdf', 'URDF_Microproyecto_9.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # 1. robot_state_publisher
    rsp = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    # 2. Lanzar Gazebo Ignition (mundo vacío)
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
        output='screen'
    )

    # 3. Insertar el robot en Gazebo
    spawn = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mi_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.05',
        ],
        output='screen',
    )

    # 4. Bridge ROS2 ↔ Gazebo Ignition
    bridge = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
        ],
        output='screen',
    )

    return launch.LaunchDescription([
        rsp,
        gz_sim,
        launch.actions.TimerAction(period=2.0, actions=[spawn]),
        launch.actions.TimerAction(period=3.0, actions=[bridge]),
    ])
