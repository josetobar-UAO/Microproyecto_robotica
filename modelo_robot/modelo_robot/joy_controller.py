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

Controles:
  LB + stick izq  → mover robot
  RB + stick izq  → turbo
  A               → reversa fija (con giro)
  B               → freno total
════════════════════════════════════════════════════════════
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyController(Node):
    def __init__(self):
        super().__init__('joy_controller')

        self.declare_parameter('scale_linear',       0.3)
        self.declare_parameter('scale_linear_turbo', 0.5)
        self.declare_parameter('scale_angular',      1.5)
        self.declare_parameter('reverse_speed',      0.25)
        self.declare_parameter('deadzone',           0.25)

        # Mapeo Xbox USB verificado
        self.AXIS_LINEAR  = 1   # stick izq vertical
        self.AXIS_ANGULAR = 0   # stick izq horizontal
        self.BTN_ENABLE   = 4   # LB
        self.BTN_TURBO    = 5   # RB
        self.BTN_REVERSE  = 0   # A = reversa
        self.BTN_BRAKE    = 1   # B = freno

        # axes[0] izq=+1 → angular.z debe ser + para girar izq en ROS
        # Con URDF v8 (joints swapeados), el signo correcto es +1.0
        self.ANGULAR_SIGN = -1.0

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info('═══════════════════════════════════')
        self.get_logger().info('joy_controller activo')
        self.get_logger().info('LB + stick izq = mover')
        self.get_logger().info('RB + stick izq = turbo')
        self.get_logger().info('A = reversa  |  B = freno')
        self.get_logger().info('═══════════════════════════════════')

    def apply_deadzone(self, value: float) -> float:
        dz = self.get_parameter('deadzone').value
        if abs(value) < dz:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - dz) / (1.0 - dz)

    def joy_callback(self, msg: Joy):
        twist = Twist()

        btn_enable  = msg.buttons[self.BTN_ENABLE]
        btn_turbo   = msg.buttons[self.BTN_TURBO]
        btn_reverse = msg.buttons[self.BTN_REVERSE]
        btn_brake   = msg.buttons[self.BTN_BRAKE]

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
        twist.linear.x  =  -raw_lin * scale_lin

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
