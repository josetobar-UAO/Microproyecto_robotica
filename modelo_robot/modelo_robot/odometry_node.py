#!/usr/bin/env python3
"""
odometry_node.py — Odometria por estima (dead reckoning) con IMU
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
El robot NO tiene encoders. Esta odometria estima la pose
integrando dos fuentes:

  - Posicion (x, y): se integra la velocidad lineal comandada en
    /cmd_vel, proyectada segun la orientacion actual.
        x += v * cos(theta) * dt
        y += v * sin(theta) * dt

  - Orientacion (theta / yaw): se toma del quaternion de
    /robot_imu, donde el giroscopio del MPU6050 ya integro el
    yaw real. Esto es mas fiable que integrar la velocidad
    angular comandada, porque el robot no gira exactamente lo
    que se le ordena (derrape de ruedas, asimetria de motores).

Por que NO se usa el acelerometro para la posicion: la doble
integracion de la aceleracion acumula deriva cuadratica y en un
IMU de bajo costo se vuelve inutil en segundos. Por eso la
posicion viene de cmd_vel y el IMU solo aporta la orientacion.

Publica:
  /odom  (nav_msgs/Odometry)      — pose completa (x, y, theta)
  TF: odom -> base_link

Si /robot_imu no llega (IMU desconectado), el nodo usa como
respaldo la velocidad angular de /cmd_vel para estimar theta:
la odometria degrada pero no se detiene.

Uso:
  ros2 run modelo_robot odometry_node
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf2_ros


def yaw_from_quaternion(q: Quaternion) -> float:
    """Extrae el angulo de yaw (rotacion en Z) de un quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Construye un quaternion de rotacion pura en Z a partir del yaw."""
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')

        # ── Parametros ───────────────────────────────────────────
        # frames
        self.declare_parameter('odom_frame',  'odom')
        self.declare_parameter('base_frame',  'base_link')
        # frecuencia de integracion / publicacion de odometria
        self.declare_parameter('publish_rate', 50.0)
        # si no llega IMU durante este tiempo (s), se usa el yaw
        # estimado desde cmd_vel como respaldo
        self.declare_parameter('imu_timeout',  0.5)

        self.odom_frame   = self.get_parameter('odom_frame').value
        self.base_frame   = self.get_parameter('base_frame').value
        rate              = self.get_parameter('publish_rate').value
        self.imu_timeout  = self.get_parameter('imu_timeout').value

        # ── Estado de la pose ────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0          # yaw actual usado para la odometria

        # comando de velocidad mas reciente
        self.v_cmd = 0.0          # velocidad lineal  (m/s)
        self.w_cmd = 0.0          # velocidad angular (rad/s) — respaldo

        # yaw proveniente del IMU
        self.imu_yaw       = None         # None hasta el primer mensaje
        self.imu_yaw_ref   = None         # yaw del IMU en el primer dato
        self.last_imu_time = None

        # ── Publishers / subscribers ─────────────────────────────
        qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, qos)
        self.create_subscription(Imu,   'robot_imu', self.imu_cb, qos)

        # ── Timer de integracion ─────────────────────────────────
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate, self.update)

        self.get_logger().info(
            'odometry_node iniciado — dead reckoning (cmd_vel) + yaw del IMU')
        self.get_logger().info(
            f'Publica /odom y TF {self.odom_frame} -> {self.base_frame}')

    # ── Callback /cmd_vel ────────────────────────────────────────
    def cmd_cb(self, msg: Twist):
        self.v_cmd = msg.linear.x
        self.w_cmd = msg.angular.z

    # ── Callback /robot_imu ──────────────────────────────────────
    def imu_cb(self, msg: Imu):
        yaw = yaw_from_quaternion(msg.orientation)
        # El primer yaw del IMU se toma como referencia 0, para que
        # la odometria empiece con theta = 0 sin importar como este
        # orientado fisicamente el sensor al arrancar.
        if self.imu_yaw_ref is None:
            self.imu_yaw_ref = yaw
        self.imu_yaw = yaw - self.imu_yaw_ref
        self.last_imu_time = self.get_clock().now()

    # ── Integracion periodica ────────────────────────────────────
    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        # ── Orientacion ──────────────────────────────────────────
        # Preferir el yaw del IMU. Si no hay IMU reciente, integrar
        # la velocidad angular comandada como respaldo.
        imu_fresh = False
        if self.imu_yaw is not None and self.last_imu_time is not None:
            age = (now - self.last_imu_time).nanoseconds * 1e-9
            imu_fresh = age < self.imu_timeout

        if imu_fresh:
            self.theta = self.imu_yaw
        else:
            # respaldo: dead reckoning puro de la rotacion
            self.theta += self.w_cmd * dt

        # normalizar theta a (-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # ── Posicion ─────────────────────────────────────────────
        # Se integra la velocidad lineal comandada proyectada con
        # la orientacion actual. Modelo de robot diferencial: sin
        # deslizamiento lateral (v_y = 0 en el marco del robot).
        self.x += self.v_cmd * math.cos(self.theta) * dt
        self.y += self.v_cmd * math.sin(self.theta) * dt

        self.publish(now, imu_fresh)

    # ── Publicacion de /odom y TF ────────────────────────────────
    def publish(self, stamp_time, imu_fresh):
        stamp = stamp_time.to_msg()
        q = quaternion_from_yaw(self.theta)

        # ── TF odom -> base_link ─────────────────────────────────
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id  = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # ── /odom ────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.position.z  = 0.0
        odom.pose.pose.orientation = q

        # twist: velocidades en el marco del robot
        odom.twist.twist.linear.x  = self.v_cmd
        odom.twist.twist.angular.z = self.w_cmd

        # Covarianzas (diagonal). La incertidumbre de posicion es
        # alta porque es dead reckoning sin encoders; el yaw es
        # mas confiable cuando proviene del IMU.
        odom.pose.covariance[0]  = 0.05    # x
        odom.pose.covariance[7]  = 0.05    # y
        odom.pose.covariance[35] = 0.02 if imu_fresh else 0.20  # yaw

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
