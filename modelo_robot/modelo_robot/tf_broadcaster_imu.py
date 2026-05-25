#!/usr/bin/env python3
"""
tf_broadcaster_imu.py — ROS 2 (Humble)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Publica UNICAMENTE el TF base_link -> imu_link con la orientacion
del IMU.

IMPORTANTE — cambio respecto a la version anterior:
La version previa publicaba TAMBIEN /odom y el TF odom -> imu_link,
con la posicion fija en (0,0,0). Eso entraba en conflicto con
odometry_node.py, que es ahora el responsable de la odometria
real (pose x, y, theta) y del TF odom -> base_link.

Reparto de responsabilidades:
  odometry_node.py      ->  /odom  y  TF odom -> base_link
  tf_broadcaster_imu.py ->  TF base_link -> imu_link  (solo sensor)

Asi el arbol de TF queda coherente:
  odom -> base_link -> imu_link

Nota: el quaternion de /robot_imu se calcula en el firmware con un
filtro complementario (pitch/roll) e integracion del giroscopio
(yaw). No proviene del DMP del MPU6050.

Uso:
  ros2 run modelo_robot tf_broadcaster_imu
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

import tf2_ros


class TfBroadcasterImu(Node):

    def __init__(self):
        super().__init__('tf_broadcaster_imu')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame',  'imu_link')
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame  = self.get_parameter('imu_frame').value

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Imu, 'robot_imu', self.imu_callback, 10)

        self.get_logger().info(
            f'tf_broadcaster_imu iniciado — TF {self.base_frame} -> {self.imu_frame}')

    def imu_callback(self, msg: Imu):
        # Tiempo del sistema: el stamp del ESP32 puede llegar en 0
        # si micro-ROS aun no sincronizo el reloj.
        now = self.get_clock().now().to_msg()
        q = msg.orientation

        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = self.base_frame
        t.child_frame_id  = self.imu_frame

        # imu_link es solidario a base_link: sin desplazamiento.
        # (si el IMU esta fisicamente desplazado del centro del
        #  robot, ese offset deberia venir del URDF, no de aqui).
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TfBroadcasterImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
