#!/usr/bin/env python3
"""
mjpeg_to_ros.py
Consume el stream MJPEG de la ESP32-CAM y publica en ROS2:
  /camera/image_raw     (sensor_msgs/Image)
  /camera/camera_info   (sensor_msgs/CameraInfo)

Uso:
  ros2 run modelo_robot mjpeg_to_ros \
    --ros-args -p stream_url:=http://192.168.X.X:81/stream

Visualizar en RViz2:
  Agregar display tipo "Image", topic "/camera/image_raw"
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import urllib.request
import numpy as np
import cv2


class MjpegToRos(Node):
    def __init__(self):
        super().__init__('mjpeg_to_ros')

        self.declare_parameter('stream_url', 'http://172.20.10.5:81/stream')
        self.declare_parameter('frame_id',   'camera_link')
        self.declare_parameter('width',      640)
        self.declare_parameter('height',     480)

        self.pub_img  = self.create_publisher(Image,      'camera/image_raw',   10)
        self.pub_info = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        url = self.get_parameter('stream_url').value
        self.get_logger().info(f'Conectando a {url}')
        self.get_logger().info('Publicando en /camera/image_raw')

        # Conectar al stream en un timer para no bloquear el constructor
        self.stream  = None
        self.bytes_  = b''
        self.create_timer(0.1, self.connect_and_read)
        self._connected = False

    def connect_and_read(self):
        if not self._connected:
            try:
                url = self.get_parameter('stream_url').value
                req = urllib.request.urlopen(url, timeout=15)
                self.stream = req
                self._connected = True
                self.get_logger().info('Stream conectado OK')
            except Exception as e:
                self.get_logger().warn(f'Reintentando conexion: {e}')
                return

        try:
            chunk = self.stream.read(4096)
            if not chunk:
                self._connected = False
                return
            self.bytes_ += chunk

            # Buscar inicio y fin del frame JPEG
            a = self.bytes_.find(b'\xff\xd8')  # SOI JPEG
            b = self.bytes_.find(b'\xff\xd9')  # EOI JPEG

            if a != -1 and b != -1 and b > a:
                jpg = self.bytes_[a:b+2]
                self.bytes_ = self.bytes_[b+2:]
                self.publish_frame(jpg)

        except Exception as e:
            self.get_logger().warn(f'Error stream: {e}')
            self._connected = False

    def publish_frame(self, jpg_bytes: bytes):
        arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        # BGR → RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, _ = frame_rgb.shape

        now = self.get_clock().now().to_msg()
        fid = self.get_parameter('frame_id').value

        # Image msg
        img_msg = Image()
        img_msg.header.stamp    = now
        img_msg.header.frame_id = fid
        img_msg.height    = h
        img_msg.width     = w
        img_msg.encoding  = 'rgb8'
        img_msg.step      = w * 3
        img_msg.data      = frame_rgb.tobytes()
        self.pub_img.publish(img_msg)

        # CameraInfo basica (sin calibracion)
        info = CameraInfo()
        info.header = img_msg.header
        info.width  = w
        info.height = h
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [1.0, 0.0, w/2.0,
                  0.0, 1.0, h/2.0,
                  0.0, 0.0, 1.0]
        self.pub_info.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = MjpegToRos()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
