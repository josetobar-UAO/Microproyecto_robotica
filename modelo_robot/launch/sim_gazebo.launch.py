#!/usr/bin/env python3
"""
sim_gazebo.launch.py — SOLO SIMULACION
════════════════════════════════════════════════════════════
Lanza el robot en Gazebo Ignition con teleop por teclado.
NO arranca el micro_ros_agent ni el joy_controller: este
launch es exclusivamente para simulacion.

  ros2 launch modelo_robot sim_gazebo.launch.py

El /cmd_vel de este launch lo consume el ros_gz_bridge y
mueve el robot simulado. No interfiere con el robot fisico
porque el robot fisico se lanza en su propio launch aparte.
════════════════════════════════════════════════════════════
"""
import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch.actions import ExecuteProcess, TimerAction


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
        output='screen',
    )

    spawn = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mi_robot',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.08',
        ],
        output='screen',
    )

    # El bridge se suscribe a /cmd_vel y lo reenvia a Gazebo.
    # Solo existe en el launch de simulacion.
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
        TimerAction(period=2.0, actions=[spawn]),
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=4.0, actions=[teleop]),
    ])
