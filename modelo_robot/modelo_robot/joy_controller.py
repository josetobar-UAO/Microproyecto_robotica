#!/usr/bin/env python3
"""
joy_controller.py — Control Xbox USB para robot diferencial
════════════════════════════════════════════════════════════
Mapeo verificado con ros2 topic echo /joy:
  axes[0]    = stick izq horizontal  izq=+1  der=-1
  axes[1]    = stick izq vertical    arr=+1  abajo=-1
  buttons[4] = LB
  buttons[5] = RB
  buttons[0] = A
  buttons[1] = B
  buttons[2] = X  → toggle flash ESP32-CAM
  buttons[3] = Y

Controles:
  LB + stick izq  → mover robot
  RB + stick izq  → turbo
  A               → reversa fija (con giro)
  B               → freno total
  X               → toggle flash ESP32-CAM (on/off)
════════════════════════════════════════════════════════════
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import urllib.request
import threading


class JoyController(Node):
    def __init__(self):
        super().__init__('joy_controller')

        self.declare_parameter('scale_linear',       0.3)
        self.declare_parameter('scale_linear_turbo', 0.5)
        self.declare_parameter('scale_angular',      1.5)
        self.declare_parameter('reverse_speed',      0.25)
        self.declare_parameter('deadzone',           0.25)
        # IP de la ESP32-CAM — ACTUALIZAR a la IP del router (192.168.1.x)
        # La 172.20.10.x era del hotspot. Revisa la IP real de la camara.
        self.declare_parameter('cam_ip', '192.168.1.5')

        # Mapeo Xbox USB verificado
        self.AXIS_LINEAR  = 1   # stick izq vertical
        self.AXIS_ANGULAR = 0   # stick izq horizontal
        self.BTN_ENABLE   = 4   # LB
        self.BTN_TURBO    = 5   # RB
        self.BTN_REVERSE  = 0   # A = reversa
        self.BTN_BRAKE    = 1   # B = freno
        self.BTN_FLASH    = 2   # X = toggle flash camara

        self.ANGULAR_SIGN = -1.0

        # Estado del flash
        self.flash_on       = False
        self.btn_flash_prev = 0   # para detectar flanco ascendente

        # QoS explicito para cmd_vel — RELIABLE/VOLATILE/KEEP_LAST.
        # Debe coincidir con el subscriber de la ESP32 (micro-ROS),
        # que tambien usa RELIABLE. Asi el emparejamiento DDS es
        # garantizado y los Twist llegan siempre al robot fisico.
        cmd_vel_qos = QoSProfile(depth=10)
        cmd_vel_qos.reliability = ReliabilityPolicy.RELIABLE
        cmd_vel_qos.durability  = DurabilityPolicy.VOLATILE
        cmd_vel_qos.history     = HistoryPolicy.KEEP_LAST

        self.pub = self.create_publisher(Twist, 'cmd_vel', cmd_vel_qos)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info('═══════════════════════════════════')
        self.get_logger().info('joy_controller activo')
        self.get_logger().info('LB + stick izq = mover')
        self.get_logger().info('RB + stick izq = turbo')
        self.get_logger().info('A = reversa  |  B = freno')
        self.get_logger().info('X = toggle flash ESP32-CAM')
        self.get_logger().info('cmd_vel QoS: RELIABLE')
        self.get_logger().info('═══════════════════════════════════')

    def apply_deadzone(self, value: float) -> float:
        dz = self.get_parameter('deadzone').value
        if abs(value) < dz:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - dz) / (1.0 - dz)

    def toggle_flash(self):
        """Llama al endpoint HTTP de la ESP32-CAM para toggle del flash."""
        self.flash_on = not self.flash_on
        ip  = self.get_parameter('cam_ip').value
        val = 1 if self.flash_on else 0
        url = f'http://{ip}/flash?val={val}'

        def http_call():
            try:
                urllib.request.urlopen(url, timeout=10)
                estado = 'ON' if self.flash_on else 'OFF'
                self.get_logger().info(f'Flash camara: {estado}')
            except Exception as e:
                self.get_logger().warn(f'Flash HTTP error: {e}')

        # Llamada en hilo separado para no bloquear el callback del joystick
        threading.Thread(target=http_call, daemon=True).start()

    def joy_callback(self, msg: Joy):
        twist = Twist()

        btn_enable  = msg.buttons[self.BTN_ENABLE]
        btn_turbo   = msg.buttons[self.BTN_TURBO]
        btn_reverse = msg.buttons[self.BTN_REVERSE]
        btn_brake   = msg.buttons[self.BTN_BRAKE]
        btn_flash   = msg.buttons[self.BTN_FLASH]

        # Detectar flanco ascendente del boton X (evita toggle rapido repetido)
        if btn_flash == 1 and self.btn_flash_prev == 0:
            self.toggle_flash()
        self.btn_flash_prev = btn_flash

        # B = freno inmediato siempre
        if btn_brake:
            self.pub.publish(twist)
            return

        # Sin LB no hay movimiento (seguridad)
        if not btn_enable:
            self.pub.publish(twist)
            return

        scale_lin = (self.get_parameter('scale_linear_turbo').value
                     if btn_turbo
                     else self.get_parameter('scale_linear').value)
        scale_ang = self.get_parameter('scale_angular').value
        rev_speed = self.get_parameter('reverse_speed').value

        raw_lin = self.apply_deadzone(msg.axes[self.AXIS_LINEAR])
        raw_ang = self.apply_deadzone(msg.axes[self.AXIS_ANGULAR])

        # A = reversa fija con giro
        if btn_reverse:
            twist.linear.x  = -rev_speed
            twist.angular.z =  raw_ang * scale_ang * self.ANGULAR_SIGN
            self.pub.publish(twist)
            return

        # Movimiento normal
        twist.linear.x  = -raw_lin * scale_lin
        twist.angular.z =  raw_ang * scale_ang * self.ANGULAR_SIGN
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
