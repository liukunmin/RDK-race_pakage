
#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage

from std_msgs.msg import String

from cv_bridge import CvBridge

import cv2

import os

import sys

import termios

import tty

import select

from datetime import datetime



class CaptureAndPublish(Node):

    def __init__(self):

        super().__init__('capture_and_publish')

        self.bridge = CvBridge()

        self.latest_image = None

        self.image_received = False

        self.camera_topic = "/image"

        qos_profile = rclpy.qos.QoSProfile(

            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,

            durability=rclpy.qos.DurabilityPolicy.VOLATILE,

            depth=10

        )

        self.sub_image = self.create_subscription(

            CompressedImage, self.camera_topic, self.image_callback, qos_profile)

        self.pub_sign_image = self.create_publisher(CompressedImage, '/sign_image', 10)

        self.result_sub = self.create_subscription(

            String, '/sign_result', self.result_callback, 10)



        self.get_logger().info("=" * 60)

        self.get_logger().info("📷 小车拍照发布节点已启动")

        self.get_logger().info(f"📡 订阅摄像头: {self.camera_topic}")

        self.get_logger().info(f"📡 发布到: /sign_image")

        self.get_logger().info("=" * 60)

        self.get_logger().info("按 Enter 拍照，按 q 退出")



    def image_callback(self, msg):

        self.latest_image = msg

        self.image_received = True



    def result_callback(self, msg):

        self.get_logger().warn(f"📢 识别结果: {msg.data}")



    def capture_and_publish(self):

        if not self.image_received or self.latest_image is None:

            self.get_logger().warn("❌ 还没有收到摄像头图像")

            return False



        self.get_logger().info("📸 拍照中...")

        try:

            cv_image = self.bridge.compressed_imgmsg_to_cv2(self.latest_image, 'bgr8')

            save_dir = os.path.expanduser("~/桌面/captured_images")

            os.makedirs(save_dir, exist_ok=True)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

            save_path = os.path.join(save_dir, f"capture_{timestamp}.jpg")

            cv2.imwrite(save_path, cv_image)

            self.get_logger().info(f"💾 保存: {save_path}")

            self.pub_sign_image.publish(self.latest_image)

            self.get_logger().info("📤 图像已发布到 /sign_image")

            return True

        except Exception as e:

            self.get_logger().error(f"❌ 拍照失败: {e}")

            return False



def get_key():

    """获取单个按键，不回车"""

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

    node = CaptureAndPublish()



    print("\n📷 按 Enter 键拍照，按 q 退出\n")



    while rclpy.ok():

        rclpy.spin_once(node, timeout_sec=0.05)



        if select.select([sys.stdin], [], [], 0)[0]:

            key = get_key()

            if key == 'q' or key == 'Q':

                break

            elif key == '\n' or key == '\r':

                node.capture_and_publish()

            else:

                # 忽略其他按键

                pass



    node.destroy_node()

    rclpy.shutdown()

    print("\n✅ 已退出")



if __name__ == '__main__':

    main()

