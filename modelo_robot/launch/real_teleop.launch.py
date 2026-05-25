#!/usr/bin/env python3
"""
real_teleop.launch.py — SOLO ROBOT FISICO
════════════════════════════════════════════════════════════
Teleoperacion del robot real via micro-ROS.
NO arranca Gazebo ni el ros_gz_bridge: por eso /cmd_vel
queda libre y lo consume UNICAMENTE el ESP32.

Esto resuelve el cmd_rx=0: cuando Gazebo + bridge corrian
en paralelo, el bridge se suscribia a /cmd_vel y acaparaba
el flujo; el cliente micro-ROS nunca recibia los Twist.

  ros2 launch modelo_robot real_teleop.launch.py
  ros2 launch modelo_robot real_teleop.launch.py cam_ip:=192.168.1.50

Componentes:
  1. micro_ros_agent  udp4 :8888   ← puente con el ESP32
  2. robot_state_publisher        ← TF del robot real
  3. joy_node                     ← Xbox  -> /joy
  4. joy_controller               ← /joy  -> /cmd_vel
  5. tf_broadcaster_imu           ← TF base_link -> imu_link
  5b. odometry_node               ← /cmd_vel + /robot_imu -> /odom
  6. mjpeg_to_ros                 ← stream ESP32-CAM -> /camera/image_raw

IMPORTANTE: pasa la IP real de la ESP32-CAM con cam_ip:= .
La IP aparece en el monitor serial de la camara al arrancar
([OK] IP: ...). Si la camara cambia de IP al reconectar al
WiFi, relanza con el nuevo cam_ip; no hay que recompilar nada.

Ver el video:  rviz2  -> Add -> Image -> topic /camera/image_raw
           o:  ros2 run rqt_image_view rqt_image_view
════════════════════════════════════════════════════════════
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('modelo_robot')
    urdf_file = os.path.join(pkg, 'urdf', 'URDF_Microproyecto_9.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    cam_ip_arg = DeclareLaunchArgument(
        'cam_ip',
        default_value='192.168.1.5',
        description='IP de la ESP32-CAM en la red del router')
    cam_ip = LaunchConfiguration('cam_ip')

    # 1. micro-ROS agent — puente UDP con el ESP32.
    #    use_sim_time = false: el robot real usa reloj de pared.
    #
    #    RMW_IMPLEMENTATION=rmw_fastrtps_cpp: el agente y todos los
    #    nodos ROS DEBEN usar el mismo RMW. El micro_ros_agent de
    #    ~/uros_ws esta compilado contra Fast-DDS; si la terminal
    #    donde corre ros2 topic pub usa otro RMW (p.ej. cyclonedds),
    #    las entidades DDS que crea el agente quedan invisibles para
    #    el grafo ROS: Subscription count: 0 y el ESP32 no recibe
    #    nada aunque el agente diga 'session established'.
    #    Exporta tambien esto en tu ~/.bashrc para todas las
    #    terminales:  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    agent = ExecuteProcess(
        cmd=['bash', '-c',
             'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && '
             'source ~/uros_ws/install/setup.bash && '
             'ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4'],
        output='screen')

    # 2. robot_state_publisher — TF del robot fisico.
    #    use_sim_time = False (sin Gazebo no hay /clock simulado).
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
    )

    # 3. Xbox -> /joy
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }])

    # 4. joy_controller -> /cmd_vel (lo recibe el ESP32)
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
            'cam_ip':              cam_ip,
        }],
        output='screen')

    # 5. IMU broadcaster — TF base_link -> imu_link (solo sensor)
    imu_broadcaster = Node(
        package='modelo_robot',
        executable='tf_broadcaster_imu',
        output='screen')

    # 5b. Odometria — dead reckoning (cmd_vel) + yaw del IMU.
    #     Publica /odom y el TF odom -> base_link. Es el nodo
    #     responsable de la pose del robot (x, y, theta).
    odometry = Node(
        package='modelo_robot',
        executable='odometry_node',
        name='odometry_node',
        parameters=[{
            'odom_frame':   'odom',
            'base_frame':   'base_link',
            'publish_rate': 50.0,
            'imu_timeout':  0.5,
        }],
        output='screen')

    # 6. Camara — consume el stream MJPEG de la ESP32-CAM y lo
    #    publica en /camera/image_raw (visualizable en RViz2).
    #    La IP se pasa con cam_ip:= ; el stream vive en el
    #    puerto 81. Si la ESP32-CAM cambia de IP al reconectar
    #    al WiFi, solo se relanza con otro cam_ip, sin recompilar.
    camera = Node(
        package='modelo_robot',
        executable='mjpeg_to_ros',
        name='mjpeg_to_ros',
        parameters=[{
            'stream_url': ['http://', cam_ip, ':81/stream'],
            'frame_id':   'camera_link',
            'width':      640,
            'height':     480,
        }],
        output='screen')

    return LaunchDescription([
        cam_ip_arg,
        agent,
        rsp,
        joy_node,
        # joy_controller arranca tras el agente para que /cmd_vel
        # ya tenga al ESP32 emparejado cuando empiece a publicar.
        TimerAction(period=3.0, actions=[joy_controller]),
        TimerAction(period=4.0, actions=[imu_broadcaster]),
        TimerAction(period=4.0, actions=[odometry]),
        # La camara arranca al final: da tiempo a que la ESP32-CAM
        # termine de conectarse al WiFi antes del primer intento.
        TimerAction(period=5.0, actions=[camera]),
    ])
