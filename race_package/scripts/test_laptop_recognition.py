#!/usr/bin/env python3
"""
test_laptop_recognition.py - 笔记本端测试脚本
功能：
1. 订阅 /sign_image（小车发来的图像）
2. 详细日志：图片格式、分辨率、大小等
3. 保存图片到本地磁盘
4. 调用 Ollama + llava:7b 进行识别
5. 发布识别结果到 /sign_result

启动命令：
  cd ~/桌面
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=0
  python3 test_laptop_recognition.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64
import requests
import os
import time
from datetime import datetime


class TestLaptopRecognition(Node):
    def __init__(self):
        super().__init__('test_laptop_recognition')

        self.ollama_url = "http://localhost:11434/api/generate"
        self.model_name = "llava:7b"
        self.prompt = "用中文一句话描述这张图片中的内容"

        self.bridge = CvBridge()
        self.save_dir = os.path.expanduser("~/桌面/captured_images")
        os.makedirs(self.save_dir, exist_ok=True)

        self.sub_image = self.create_subscription(
            CompressedImage, '/sign_image', self.image_callback, 10)
        self.result_pub = self.create_publisher(String, '/sign_result', 10)

        self.get_logger().info("=" * 60)
        self.get_logger().info("🤖 笔记本端测试识别节点启动")
        self.get_logger().info(f"📦 模型: {self.model_name}")
        self.get_logger().info(f"🔗 Ollama URL: {self.ollama_url}")
        self.get_logger().info(f"💾 保存目录: {self.save_dir}")
        self.get_logger().info("📡 订阅: /sign_image (CompressedImage)")
        self.get_logger().info("📤 发布: /sign_result (String)")
        self.get_logger().info("=" * 60)

    def image_callback(self, msg):
        self.get_logger().info("=" * 50)
        self.get_logger().info("📷 收到图像，开始处理...")
        self.get_logger().info("=" * 50)

        # ===== 1. 详细日志：ROS 消息信息 =====
        self.get_logger().info(f"[MSG] 话题: /sign_image")
        self.get_logger().info(f"[MSG] 消息类型: CompressedImage")
        self.get_logger().info(f"[MSG] format: {msg.format}")
        self.get_logger().info(f"[MSG] data 长度: {len(msg.data)} bytes ({len(msg.data)/1024:.1f} KB)")
        self.get_logger().info(f"[MSG] header.stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        self.get_logger().info(f"[MSG] header.frame_id: {msg.header.frame_id}")

        try:
            # ===== 2. 解码图像 =====
            t_start = time.time()
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            t_decode = time.time() - t_start

            h, w = cv_image.shape[:2]
            channels = cv_image.shape[2] if len(cv_image.shape) == 3 else 1
            dtype = cv_image.dtype
            total_pixels = h * w
            mem_size = cv_image.nbytes

            self.get_logger().info(f"[IMG] 解码耗时: {t_decode*1000:.1f} ms")
            self.get_logger().info(f"[IMG] 分辨率: {w}x{h}")
            self.get_logger().info(f"[IMG] 通道数: {channels}")
            self.get_logger().info(f"[IMG] 数据类型: {dtype}")
            self.get_logger().info(f"[IMG] 总像素: {total_pixels:,}")
            self.get_logger().info(f"[IMG] 内存占用: {mem_size/1024:.1f} KB ({mem_size/1024/1024:.2f} MB)")
            self.get_logger().info(f"[IMG] 压缩比: {len(msg.data)/mem_size:.2f} (压缩后/原始)")

            # ===== 3. 保存图片 =====
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = os.path.join(self.save_dir, f"capture_{timestamp}.jpg")
            t_start = time.time()
            success = cv2.imwrite(save_path, cv_image)
            t_save = time.time() - t_start
            file_size = os.path.getsize(save_path) if success else 0

            if success:
                self.get_logger().info(f"[SAVE] 保存耗时: {t_save*1000:.1f} ms")
                self.get_logger().info(f"[SAVE] 文件路径: {save_path}")
                self.get_logger().info(f"[SAVE] 文件大小: {file_size/1024:.1f} KB")
            else:
                self.get_logger().error("[SAVE] 保存失败！")

            # ===== 4. 编码为 base64 =====
            t_start = time.time()
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            base64_image = base64.b64encode(buffer).decode('utf-8')
            t_encode = time.time() - t_start

            self.get_logger().info(f"[B64] 编码耗时: {t_encode*1000:.1f} ms")
            self.get_logger().info(f"[B64] base64 长度: {len(base64_image)} chars ({len(base64_image)/1024:.1f} KB)")

            # ===== 5. 调用 Ollama API =====
            self.get_logger().info(f"[OLLAMA] 模型: {self.model_name}")
            self.get_logger().info(f"[OLLAMA] Prompt: {self.prompt}")
            self.get_logger().info(f"[OLLAMA] 正在请求识别...")

            payload = {
                "model": self.model_name,
                "prompt": self.prompt,
                "images": [base64_image],
                "stream": False
            }

            t_start = time.time()
            response = requests.post(self.ollama_url, json=payload, timeout=60)
            t_infer = time.time() - t_start

            self.get_logger().info(f"[OLLAMA] 响应状态码: {response.status_code}")
            self.get_logger().info(f"[OLLAMA] 推理耗时: {t_infer:.2f} s")

            if response.status_code == 200:
                result = response.json()
                description = result.get('response', '').strip()
                eval_count = result.get('eval_count', 0)
                eval_duration = result.get('eval_duration', 0)

                self.get_logger().info(f"[OLLAMA] eval_count: {eval_count} tokens")
                if eval_duration > 0:
                    self.get_logger().info(f"[OLLAMA] eval_duration: {eval_duration/1e9:.2f} s")
                    self.get_logger().info(f"[OLLAMA] 速度: {eval_count/(eval_duration/1e9):.1f} tokens/s")
                self.get_logger().warn(f"[RESULT] 识别结果: {description}")

                result_msg = String()
                result_msg.data = description
                self.result_pub.publish(result_msg)
                self.get_logger().info(f"[PUB] 已发布到 /sign_result")
            else:
                self.get_logger().error(f"[OLLAMA] 请求失败: {response.status_code}")
                self.get_logger().error(f"[OLLAMA] 响应: {response.text[:200]}")

        except requests.exceptions.ConnectionError:
            self.get_logger().error("❌ 无法连接到 Ollama 服务！请确保已启动: ollama serve")
        except Exception as e:
            self.get_logger().error(f"❌ 处理异常: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

        self.get_logger().info("=" * 50)


def main():
    rclpy.init()
    node = TestLaptopRecognition()

    print("\n🤖 等待小车发送图像...\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    print("\n✅ 已退出\n")


if __name__ == '__main__':
    main()