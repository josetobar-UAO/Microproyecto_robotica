import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch.actions import ExecuteProcess


def generate_launch_description():

    pkg = get_package_share_directory('modelo_robot')
    urdf_file = os.path.join(pkg, 'urdf', 'URDF_Microproyecto_9.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    rsp = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
        output='screen'
    )

    # Sin rotacion extra — los ejes del URDF ya estan corregidos
    spawn = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mi_robot',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.05',
        ],
        output='screen',
    )

    bridge = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen',
    )

    teleop = launch_ros.actions.Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
    )

    return launch.LaunchDescription([
        rsp,
        gz_sim,
        launch.actions.TimerAction(period=2.0, actions=[spawn]),
        launch.actions.TimerAction(period=3.0, actions=[bridge]),
        launch.actions.TimerAction(period=4.0, actions=[teleop]),
    ])
