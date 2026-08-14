#!/usr/bin/env python3
"""
sign_trigger.py - 大模型识别触发节点（小车端）
功能：
1. 从 YellowChannel 获取路径点列表，取索引3作为立牌点
2. 通过 TF 检测小车是否到达立牌点
3. 到达后拍照，发布到 /sign_image
4. 订阅 /sign_result，打印结果
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import math
import cv2
import numpy as np

from tf2_ros import TransformListener, Buffer
from config import Config


class SignTrigger(Node):
    def __init__(self, waypoints=None):
        super().__init__('sign_trigger')
        
        # ===== 从路径点获取立牌坐标 =====
        self.waypoints = waypoints
        if self.waypoints and len(self.waypoints) > 3:
            self.sign_point_x, self.sign_point_y = self.waypoints[3]
            self.get_logger().info(f"📍 立牌点: ({self.sign_point_x}, {self.sign_point_y})")
        else:
            self.sign_point_x = 3.0
            self.sign_point_y = 3.0
            self.get_logger().warn("⚠️ 使用默认立牌点 (3.0, 3.0)")
        
        self.trigger_dist = Config.SIGN_TRIGGER_DIST
        self.image_topic = "/image"  # ★★★ 改成小车的摄像头话题 ★★★
        self.triggered = False
        self.recognized = False
        self.last_result = ""
        
        # ===== TF =====
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ===== 摄像头 =====
        self.latest_image = None
        self.image_received = False
        qos_image = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.sub_image = self.create_subscription(
            CompressedImage, self.image_topic, self.image_callback, qos_image)
        
        # ===== 发布/订阅 =====
        self.image_pub = self.create_publisher(CompressedImage, Config.IMAGE_TOPIC, 10)
        self.result_sub = self.create_subscription(
            String, Config.RESULT_TOPIC, self.result_callback, 10)
        
        # ===== 定时器 =====
        self.check_timer = self.create_timer(0.2, self.check_position)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("📸 大模型识别触发节点启动")
        self.get_logger().info(f"📍 立牌点: ({self.sign_point_x}, {self.sign_point_y})")
        self.get_logger().info(f"📏 触发距离: {self.trigger_dist}m")
        self.get_logger().info(f"📷 摄像头话题: {self.image_topic}")
        self.get_logger().info("=" * 60)
    
    def image_callback(self, msg):
        """保存最新一帧图像"""
        self.latest_image = msg
        self.image_received = True
    
    def result_callback(self, msg):
        """收到识别结果"""
        if self.recognized:
            return
        self.recognized = True
        self.last_result = msg.data
        self.get_logger().warn(f"📢 大模型识别结果: {self.last_result}")
        print(f"\n{'='*60}")
        print(f"📢 立牌识别结果: {self.last_result}")
        print(f"{'='*60}\n")
    
    def get_robot_pose(self):
        """获取小车在 map 中的位置"""
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            return (trans.transform.translation.x, trans.transform.translation.y)
        except Exception:
            return (None, None)
    
    def check_position(self):
        """检查小车是否到达立牌点"""
        if self.triggered:
            return
        
        pose = self.get_robot_pose()
        if pose[0] is None:
            return
        
        x, y = pose
        dist = math.hypot(x - self.sign_point_x, y - self.sign_point_y)
        
        if dist < self.trigger_dist:
            self.get_logger().warn(f"📸 到达立牌点！距离={dist:.2f}m，触发拍照")
            self.triggered = True
            self._trigger_recognition()
    
    def _trigger_recognition(self):
        """触发大模型识别：拍照 → 裁剪 → 发送到笔记本"""
        if not self.image_received or self.latest_image is None:
            self.get_logger().warn("❌ 没有收到摄像头图像，无法触发识别")
            self.triggered = False
            return

        # 裁剪图片：去掉上方1/4、下方1/4、左右各1/6
        cropped_msg = self._crop_image(self.latest_image)

        self.get_logger().info("📷 正在发送裁剪后图像到笔记本...")
        self.image_pub.publish(cropped_msg)
        self.get_logger().info("📷 图像已发送，等待识别结果...")

    def _crop_image(self, msg):
        """裁剪CompressedImage：去上1/4、下1/4、左1/6、右1/6"""
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is None:
                self.get_logger().warn("⚠️ 图像解码失败，发送原图")
                return msg

            h, w = img.shape[:2]
            top = h // 4
            bottom = h * 3 // 4
            left = w // 6
            right = w * 5 // 6
            cropped = img[top:bottom, left:right]

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
            _, encoded = cv2.imencode('.jpg', cropped, encode_param)

            out_msg = CompressedImage()
            out_msg.header = msg.header
            out_msg.format = "jpeg"
            out_msg.data = encoded.tobytes()
            self.get_logger().info(f"✂️ 裁剪: {w}x{h} → {right-left}x{bottom-top}")
            return out_msg
        except Exception as e:
            self.get_logger().warn(f"⚠️ 裁剪失败: {e}，发送原图")
            return msg


def main():
    rclpy.init()
    node = SignTrigger()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()