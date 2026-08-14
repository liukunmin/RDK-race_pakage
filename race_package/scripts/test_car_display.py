#!/usr/bin/env python3
"""
test_car_display.py - 小车端测试脚本
功能：
1. 订阅 /image，实时检测二维码并显示数字
2. 订阅 /sign_result，显示大模型识别结果（二维码数字下方）
3. 按 Enter 手动拍照，发布到 /sign_image
4. 屏幕一直显示二维码数字 + 识别结果

启动命令：
  cd /origin_ws
  source install/setup.bash
  export ROS_DOMAIN_ID=0
  python3 src/race_package/scripts/test_car_display.py
"""

import os
import sys
sys.path.insert(0, '/usr/lib/python3/dist-packages')
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from display_utils import draw_display, set_fullscreen
os.environ.setdefault('DISPLAY', ':0')
os.environ.setdefault('XAUTHORITY', '/home/sunrise/.Xauthority')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import select
import termios
import tty

# 微信二维码模型路径
_WECHAT_QR_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              '..', '..', '..', 'models', 'opencv_3rdparty-wechat_qrcode')


class TestCarDisplay(Node):
    def __init__(self):
        super().__init__('test_car_display')

        self.qr_detector = cv2.wechat_qrcode_WeChatQRCode(
            os.path.join(_WECHAT_QR_DIR, 'detect.prototxt'),
            os.path.join(_WECHAT_QR_DIR, 'detect.caffemodel'),
            os.path.join(_WECHAT_QR_DIR, 'sr.prototxt'),
            os.path.join(_WECHAT_QR_DIR, 'sr.caffemodel'))
        self.qr_detected = False
        self.qr_last_result = "Waiting QR..."
        self.qr_direction = ""
        self.sign_result_text = ""
        self.latest_image = None
        self.frame_count = 0

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.sub_image = self.create_subscription(
            CompressedImage, '/image', self.image_callback, qos)
        self.sub_sign_result = self.create_subscription(
            String, '/sign_result', self.sign_result_callback, 10)
        self.image_pub = self.create_publisher(CompressedImage, '/sign_image', 10)

        cv2.namedWindow('Display', cv2.WINDOW_NORMAL)
        self._fullscreen_set = False

        self.get_logger().info("=" * 60)
        self.get_logger().info("📺 小车端测试显示节点启动")
        self.get_logger().info("📡 订阅: /image, /sign_result")
        self.get_logger().info("📤 发布: /sign_image")
        self.get_logger().info("⌨  按 Enter 拍照发送，按 q 退出")
        self.get_logger().info("=" * 60)

    def image_callback(self, msg):
        self.latest_image = msg
        self.frame_count += 1

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return

            if not self.qr_detected and self.frame_count % 3 == 0:
                res, _ = self.qr_detector.detectAndDecode(frame)
                data = res[0] if res else ""
                if data:
                    self.qr_detected = True
                    self.qr_last_result = data
                    try:
                        number = int(data.strip())
                        self.qr_direction = "顺时针" if number % 2 == 1 else "逆时针"
                    except ValueError:
                        self.qr_direction = ""
                    self.get_logger().warn(f"[QR] 检测到二维码: {data} → {self.qr_direction}")

            self._update_display()
        except Exception as e:
            self.get_logger().warning(f"图像异常: {e}")

    def sign_result_callback(self, msg):
        self.sign_result_text = msg.data
        self.get_logger().warn(f"[SIGN] 收到识别结果: {self.sign_result_text}")
        self._update_display()

    def _update_display(self):
        display = draw_display(self.qr_last_result, self.sign_result_text, self.qr_detected, self.qr_direction)
        display = cv2.rotate(display, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow('Display', display)
        set_fullscreen('Display')
        cv2.waitKey(1)

    def capture_and_send(self):
        if self.latest_image is None:
            self.get_logger().warn("还没有收到图像")
            return

        self.image_pub.publish(self.latest_image)
        self.get_logger().info("📸 已拍照并发布到 /sign_image")


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main():
    rclpy.init()
    node = TestCarDisplay()

    print("\n📷 按 Enter 拍照发送，按 q 退出\n")

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)

        if select.select([sys.stdin], [], [], 0)[0]:
            key = get_key()
            if key == 'q' or key == 'Q':
                break
            elif key == '\n' or key == '\r':
                node.capture_and_send()

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()