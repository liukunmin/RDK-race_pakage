#!/usr/bin/env python3
import sys
sys.path.insert(0, '/usr/lib/python3/dist-packages')
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import cv2
import numpy as np

from tf2_ros import TransformListener, Buffer

from config import Config
from smart_movement import SmartMovementMixin
from display_utils import draw_display, set_fullscreen

# 微信二维码模型路径
_WECHAT_QR_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              '..', '..', 'models', 'opencv_3rdparty-wechat_qrcode')


class BlueHall(Node, SmartMovementMixin):
    def __init__(self):
        super().__init__('blue_hall')

        self.exit_flag = False
        self.delayed_timer = None

        # ===== 导航客户端 =====
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ===== 激光雷达 =====
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)


        # ===== 二维码识别（微信二维码模型，HDMI 显示结果） =====
        self.qr_detector = cv2.wechat_qrcode_WeChatQRCode(
            os.path.join(_WECHAT_QR_DIR, 'detect.prototxt'),
            os.path.join(_WECHAT_QR_DIR, 'detect.caffemodel'),
            os.path.join(_WECHAT_QR_DIR, 'sr.prototxt'),
            os.path.join(_WECHAT_QR_DIR, 'sr.caffemodel'))
        self.qr_frame_count = 0

        self.qr_last_result = "Waiting..."
        self.sign_result_text = ""
        self.sub_image = self.create_subscription(CompressedImage, '/image', self.image_callback, 5)
        self.sub_sign_result = self.create_subscription(String, '/sign_result', self.sign_result_callback, 10)
        cv2.namedWindow('QR', cv2.WINDOW_NORMAL)
        self._fullscreen_set = False

        # ===== TF =====
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ===== 激光雷达数据 =====
        self.lidar_ranges = None
        self.lidar_angle_min = 0.0
        self.lidar_angle_increment = 0.0
        self.front_dist = 999.0
        self.front_distances = {}
        self.back_distances = {}

        # ===== 状态机 =====
        self.state = 'GO_TO_PRE_QR'
        self.sub_state = None
        self.goal_handle = None
        self.goal_sent = False
        self.qr_detected = False
        self.blue_hall_done = False
        self.goal_sent_time = None

        # ===== 路径点 =====
        self.waypoints = []
        self.direction = "counter_clockwise"

        # ===== 智能移动变量 =====
        self.is_moving = False
        self.move_timer = None
        self.move_pub_timer = None
        self.move_check_timer = None
        self.move_speed = 0.0
        self.move_steering = 0.0
        self.move_max_dist = 0.0
        self.move_start_time = None
        self.move_monitor_angles = []
        self.move_count = 0

        # ===== 圆弧调整变量 =====
        self.rotating = False
        self.rotate_direction = 'right'
        self.rotate_timer = None
        self.drive_timer = None
        self._current_cmd = (0.0, 0.0)
        self._rotation_interrupted = False
        self.cone_in_front = False
        self._rotation_cycle_count = 0

        # ===== 静止检测 =====
        self.last_stuck_position = None
        self.last_stuck_time = None
        self.stuck_attempt_count = 0

        # ===== 定时器 =====
        self.arrival_timer = self.create_timer(0.05, self.check_arrival)
        self.stuck_timer = self.create_timer(0.2, self.check_stuck)
        self.goal_timeout_timer = self.create_timer(1.0, self.check_goal_timeout)

        self.get_logger().info("=" * 60)
        self.get_logger().info("🔵 蓝色大厅模块启动")
        self.get_logger().info("=" * 60)

        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("等待 Nav2...")
        self.get_logger().info("Nav2 服务已就绪")

        self._goal_retry_count = 0
        self.first_goal_accepted = False
        self._initial_goal_timer = self.create_timer(1.0, self._send_initial_goal)

    def _send_initial_goal(self):
        if self._initial_goal_timer:
            self._initial_goal_timer.cancel()
            self._initial_goal_timer = None
        self.get_logger().info(f"[GOAL] 发送前置路径点 (重试#{self._goal_retry_count})")
        self.goal_sent = False
        self.send_goal(Config.PRE_QR_X, Config.PRE_QR_Y)

    # ==================== 二维码识别（微信二维码模型） ====================
    def sign_result_callback(self, msg):
        self.sign_result_text = msg.data
        self.get_logger().warn(f"[SIGN] 识别结果: {self.sign_result_text}")

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return

            self.qr_frame_count += 1
            if self.qr_frame_count % Config.QR_LOG_INTERVAL == 0:
                self.get_logger().info(f"[QR] 收到图像 frame={self.qr_frame_count}")
            if not self.qr_detected and self.qr_frame_count % Config.QR_DETECT_INTERVAL == 0:
                data, method = self._detect_qr(frame)
                if data:
                    self.get_logger().warn(f"[QR] 检测到二维码: {data} (方法: {method})")
                    self._on_qr_detected(data)

            direction_cn = ""
            if self.qr_detected:
                direction_cn = "顺时针" if self.direction == "clockwise" else "逆时针"
            display = draw_display(self.qr_last_result, self.sign_result_text, self.qr_detected, direction_cn)
            display = cv2.rotate(display, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imshow('QR', display)
            set_fullscreen('QR')
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.exit_flag = True
        except Exception as e:
            self.get_logger().warning(f"[QR] 异常: {e}")

    def _detect_qr(self, frame):
        """综合QR检测：原图→透视矫正→放大，返回(data, method)"""
        # 1. 原图
        data, points = self._try_detect(frame)
        if data:
            return data, "原图"
        if points is not None:
            data, _ = self._try_detect(self._correct_perspective(frame, points))
            if data:
                return data, "透视矫正"

        # 2. 放大（最多2次）
        for zoom in Config.QR_ZOOM_FACTORS:
            zoomed = self._zoom_image(frame, zoom)
            data, points = self._try_detect(zoomed)
            if data:
                return data, f"{zoom}x放大"
            if points is not None:
                data, _ = self._try_detect(self._correct_perspective(zoomed, points))
                if data:
                    return data, f"{zoom}x+透视矫正"

        return "", ""

    def _try_detect(self, frame):
        """单次检测，返回(data, points)"""
        res, points = self.qr_detector.detectAndDecode(frame)
        data = res[0] if res and res[0] else ""
        if data:
            return data, None
        # 检测到角点但解码失败
        if points is not None and len(points) > 0:
            pts = np.array(points[0], dtype=np.float32)
            if pts.shape == (4, 2):
                return "", pts
        return "", None

    def _correct_perspective(self, frame, points):
        """透视矫正：用四角点把梯形QR拉平成矩形"""
        src = np.float32(points)
        w = int(max(np.linalg.norm(src[0] - src[1]), np.linalg.norm(src[2] - src[3])))
        h = int(max(np.linalg.norm(src[0] - src[3]), np.linalg.norm(src[1] - src[2])))
        if w < Config.QR_MIN_SIZE or h < Config.QR_MIN_SIZE:
            return frame
        dst = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
        M = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(frame, M, (w, h))

    def _rotate_image(self, frame, angle):
        """旋转图像"""
        h, w = frame.shape[:2]
        M = cv2.getRotationMatrix2D((w // 2, h // 2), angle, 1.0)
        return cv2.warpAffine(frame, M, (w, h))

    def _zoom_image(self, frame, zoom_factor):
        """数字放大：裁剪中心 1/zoom_factor 区域并放大到原始尺寸"""
        h, w = frame.shape[:2]
        crop_w = int(w / zoom_factor)
        crop_h = int(h / zoom_factor)
        x = (w - crop_w) // 2
        y = (h - crop_h) // 2
        cropped = frame[y:y + crop_h, x:x + crop_w]
        zoomed = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)
        return zoomed

    def _on_qr_detected(self, data):
        if self.qr_detected:
            return
        self.get_logger().warn(f"[QR] 检测到二维码: {data}")

        try:
            number = int(data.strip())
            if number % 2 == 1:
                self.direction = "clockwise"
                self.get_logger().warn(f"[QR] {number} 是奇数 → 顺时针绕行")
            else:
                self.direction = "counter_clockwise"
                self.get_logger().warn(f"[QR] {number} 是偶数 → 逆时针绕行")
        except ValueError:
            self.get_logger().warn(f"[QR] 无法解析数字: {data}，使用默认方向")
            self.direction = "counter_clockwise"

        self.qr_detected = True
        self.qr_last_result = data

        if self.direction == "clockwise":
            self.waypoints = Config.PATH_CLOCKWISE
        else:
            self.waypoints = Config.PATH_COUNTER_CLOCKWISE
        self.get_logger().info(f"[QR] 路径点序列已加载，共{len(self.waypoints)}个点")

        self.cancel_current_goal()
        self.sub_state = None
        
        # ★★★ 恢复：取消注释，进入智能后退 → 圆弧调整链条 ★★★
        self._rotation_interrupted = True
        self.get_logger().info("[STATE] 二维码完成 → 进入智能后退")
        self.start_smart_backward(reason="二维码后")
        
        # 删除原来的直接导航代码
        # self.get_logger().info("[STATE] 二维码完成 → 直接导航到黄色通道入口")
        # self.state = 'GO_TO_ENTRY'
        # self.goal_sent = False
        # self.send_goal(self.waypoints[0][0], self.waypoints[0][1])

    # ==================== 激光雷达 ====================
    def scan_callback(self, msg):
        self.lidar_ranges = msg.ranges
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_increment = msg.angle_increment

        front_angles = list(range(0, 31)) + list(range(329, 360))
        front = [msg.ranges[i] for i in front_angles if 0.1 < msg.ranges[i] < 10]
        self.front_dist = min(front) if front else 999.0

        self.front_distances = {}
        for angle_offset in range(-60, 61, 5):
            angle_rad = math.radians(angle_offset)
            idx = int((angle_rad - self.lidar_angle_min) / self.lidar_angle_increment)
            if 0 <= idx < len(msg.ranges):
                dist = msg.ranges[idx]
                self.front_distances[angle_offset] = dist if 0.1 < dist < 10.0 else 999.0
            else:
                self.front_distances[angle_offset] = 999.0

        self.back_distances = {}
        for angle_offset in range(-60, 61, 5):
            angle_deg = 180 + angle_offset
            idx = int((math.radians(angle_deg) - self.lidar_angle_min) / self.lidar_angle_increment)
            if 0 <= idx < len(msg.ranges):
                dist = msg.ranges[idx]
                self.back_distances[angle_offset] = dist if 0.1 < dist < 10.0 else 999.0
            else:
                self.back_distances[angle_offset] = 999.0


    # ==================== 辅助函数 ====================
    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (x, y, yaw)
        except Exception:
            return (None, None, None)

    def get_angle_to_target(self, target_x, target_y):
        pose = self.get_current_pose()
        if pose[0] is None:
            return 0
        robot_x, robot_y, robot_yaw = pose
        dx = target_x - robot_x
        dy = target_y - robot_y
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - robot_yaw
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        if angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        return math.degrees(angle_diff)

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        if getattr(self, 'sub_state', None) == 'ROTATION':
            self.get_logger().info("[ROTATION] stop_robot() | linear=0.00, angular=0.00")

    def publish_drive(self, linear, angular, reason=""):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)
        if reason:
            self.get_logger().info(f"[CMD] {reason} | linear={linear:.2f}, angular={angular:.2f}")

    def cancel_current_goal(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.goal_sent = False

    def send_goal(self, x, y, yaw=0.0):
        if self.goal_sent:
            return
        self.get_logger().info(f"[GOAL] 发送目标点: ({x}, {y}), 朝向={math.degrees(yaw):.1f}°")
        self.goal_sent_time = self.get_clock().now()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_sent = True
        self._goal_response_received = False
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)
        self._goal_response_timer = self.create_timer(5.0, self._check_goal_response_timeout)

    def _check_goal_response_timeout(self):
        if hasattr(self, '_goal_response_timer') and self._goal_response_timer:
            self._goal_response_timer.cancel()
            self._goal_response_timer = None
        if getattr(self, '_goal_response_received', False):
            return
        self.get_logger().error("[GOAL] 发送后5秒无响应！重试")
        self.goal_sent = False
        self._goal_retry_timer = self.create_timer(1.0, self._retry_goal)

    def goal_response_callback(self, future):
        self._goal_response_received = True
        if hasattr(self, '_goal_response_timer') and self._goal_response_timer:
            self._goal_response_timer.cancel()
            self._goal_response_timer = None
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.goal_handle = goal_handle
                self._goal_retry_count = 0
                self.first_goal_accepted = True
                self.get_logger().info("[GOAL] 目标已接受，等待导航结果...")
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self.goal_result_callback)
            else:
                self.get_logger().error("[GOAL] 目标被拒绝！2秒后重试")
                self.goal_sent = False
                self._goal_retry_timer = self.create_timer(2.0, self._retry_goal)
        except Exception as e:
            self.get_logger().error(f"目标响应异常: {e}")
            self.goal_sent = False
            self._goal_retry_timer = self.create_timer(2.0, self._retry_goal)

    def _retry_goal(self):
        if hasattr(self, '_goal_retry_timer') and self._goal_retry_timer:
            self._goal_retry_timer.cancel()
            self._goal_retry_timer = None
        if self.blue_hall_done or self.exit_flag:
            return
        self._goal_retry_count += 1
        if self._goal_retry_count > 5:
            self.get_logger().error(f"[GOAL] 重试已达上限{self._goal_retry_count}次，放弃")
            return
        self.get_logger().info(f"[GOAL] 重试发送目标 #{self._goal_retry_count} (状态={self.state})")
        self.send_current_goal()

    def goal_result_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"[GOAL] 导航结果: status={result.status}")
        except Exception as e:
            self.get_logger().error(f"[GOAL] 导航结果异常: {e}")

    def _send_qr_goal(self):
        if hasattr(self, '_qr_goal_timer') and self._qr_goal_timer:
            self._qr_goal_timer.cancel()
            self._qr_goal_timer = None
        self.get_logger().info("[GOAL] 发送二维码目标点")
        self.send_goal(Config.QR_GOAL_X, Config.QR_GOAL_Y, Config.QR_GOAL_YAW)

    def send_current_goal(self):
        if self.blue_hall_done or self.exit_flag:
            return
        if self.state == 'GO_TO_PRE_QR':
            self.send_goal(Config.PRE_QR_X, Config.PRE_QR_Y)
        elif self.state == 'GO_TO_QR':
            self.send_goal(Config.QR_GOAL_X, Config.QR_GOAL_Y, Config.QR_GOAL_YAW)
        elif self.state == 'GO_TO_ENTRY':
            self.send_goal(self.waypoints[0][0], self.waypoints[0][1])

    # ==================== 到达检查 ====================
    def check_arrival(self):
        if self.blue_hall_done or self.exit_flag:
            return

        if self.sub_state is not None:
            return

        pose = self.get_current_pose()
        if pose[0] is None:
            return

        if self.state == 'GO_TO_PRE_QR':
            dist = math.hypot(pose[0] - Config.PRE_QR_X, pose[1] - Config.PRE_QR_Y)
            self.get_logger().info(f"[CHECK] GO_TO_PRE_QR dist={dist:.2f}m tol={Config.PRE_QR_TOLERANCE}m")
            if dist < Config.PRE_QR_TOLERANCE:
                self.get_logger().warn(f"[STATE] 到达前置点，距离={dist:.2f}m → 导航到二维码点")
                self.cancel_current_goal()
                self.state = 'GO_TO_QR'
                self.goal_sent = False
                self.send_goal(Config.QR_GOAL_X, Config.QR_GOAL_Y, Config.QR_GOAL_YAW)
            return

        if self.state == 'GO_TO_QR':
            dist = math.hypot(pose[0] - Config.QR_GOAL_X, pose[1] - Config.QR_GOAL_Y)
            if dist < Config.ARRIVAL_TOLERANCE and self.sub_state is None:
                self.get_logger().info("[STATE] 到达二维码点附近，等待检测")
                self.cancel_current_goal()
                self.sub_state = 'WAITING_QR'
                self._qr_wait_start = self.get_clock().now()
            return

        if self.state == 'GO_TO_ENTRY':
            dist = math.hypot(pose[0] - self.waypoints[0][0], pose[1] - self.waypoints[0][1])
            if dist < Config.WAYPOINT_TOLERANCES.get(0, 0.8):
                self.get_logger().info("[STATE] 到达入口！")
                # ★ 不cancel，让YellowChannel的新目标直接替换，避免cancel延迟
                self.stop_robot()
                self.exit_flag = True
                self.blue_hall_done = True

    # ==================== 目标超时 ====================
    def check_goal_timeout(self):
        if self.blue_hall_done or self.exit_flag:
            return
        if not self.goal_sent:
            return
        if self.sub_state is not None:
            return
        elapsed = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
        if elapsed > Config.GOAL_TIMEOUT:
            self._goal_retry_count += 1
            self.get_logger().warn(f"[TIMEOUT] 状态={self.state} 导航超时(重试#{self._goal_retry_count})，重发目标")
            self.cancel_current_goal()
            self.send_current_goal()


    # ==================== 圆弧调整 ====================
    def _enter_rotation(self):
        self.cancel_current_goal()

        self.sub_state = 'ROTATION'
        self.rotating = False
        self.cone_in_front = False

        self._rotation_cycle_count = 0
        self.get_logger().info("[ROTATION] 开始圆弧调整")
        self._rotate_once()

    def _check_cone_in_front(self):
        """检测车头前方 ±30° 是否有锥桶"""
        if self.lidar_ranges is None:
            return False
        
        cone_scan_low = Config.ROTATION_CONE_SCAN_LOW
        cone_scan_high = Config.ROTATION_CONE_SCAN_HIGH
        threshold = Config.ROTATION_CONE_THRESHOLD
        
        for angle_offset in range(cone_scan_low, cone_scan_high + 1, 2):
            angle_rad = math.radians(angle_offset)
            idx = int((angle_rad - self.lidar_angle_min) / self.lidar_angle_increment)
            if 0 <= idx < len(self.lidar_ranges):
                dist = self.lidar_ranges[idx]
                if 0.1 < dist < threshold:
                    self.get_logger().warn(f"[CONE] 前方 {angle_offset}° 检测到锥桶，距离 {dist:.2f}m")
                    return True
        return False

    def _rotate_once(self):
        self._rotation_cycle_count += 1
        angle_diff = self.get_angle_to_target(self.waypoints[0][0], self.waypoints[0][1])
        self.get_logger().info(f"[ROTATION] 第{self._rotation_cycle_count}轮 夹角={angle_diff:.1f}°, 目标<{Config.ANGLE_THRESHOLD}°")

        if abs(angle_diff) < Config.ANGLE_THRESHOLD:
            self._finish_rotation()
            return

        if self._rotation_cycle_count > Config.ROTATION_MAX_CYCLES:
            self.get_logger().warn(f"[ROTATION] 已达最大循环{Config.ROTATION_MAX_CYCLES}次，强制结束")
            self._finish_rotation()
            return

        if angle_diff > 0:
            self.rotate_direction = 'right'
            self._current_cmd = (Config.ROTATION_BACK_SPEED, Config.ROTATION_BACK_STEERING)
        else:
            self.rotate_direction = 'left'
            self._current_cmd = (Config.ROTATION_BACK_SPEED, -Config.ROTATION_BACK_STEERING)

        dynamic_duration = min(Config.ROTATION_BACK_DURATION,
                             max(0.2, abs(angle_diff) / 90.0 * Config.ROTATION_BACK_DURATION * 2))
        self._current_back_duration = dynamic_duration
        self._current_fwd_duration = dynamic_duration

        self._rotation_pub_count = 0
        self.get_logger().warn(
            f"[ROTATION] 向{'右' if self.rotate_direction == 'right' else '左'}转 | "
            f"后退指令: linear={self._current_cmd[0]:.2f}, angular={self._current_cmd[1]:.2f} | "
            f"动态时长={dynamic_duration:.2f}s")

        self._start_drive_timer_with_cone_check()
        
        if self.rotate_timer:
            self.rotate_timer.cancel()
        self.rotate_timer = self.create_timer(dynamic_duration, self._back_phase_done)

    def _finish_rotation(self):
        angle_diff = self.get_angle_to_target(self.waypoints[0][0], self.waypoints[0][1])
        self.get_logger().warn(f"[ROTATION] 夹角={angle_diff:.1f}° 达标 → 进入导航到入口")
        self._cleanup_rotation_timers()
        self.cancel_current_goal()
        self.sub_state = None
        self.rotating = False
        self.state = 'GO_TO_ENTRY'
        self.goal_sent = False
        self.send_goal(self.waypoints[0][0], self.waypoints[0][1])

    def _send_entry_goal(self):
        if hasattr(self, '_entry_goal_timer') and self._entry_goal_timer:
            self._entry_goal_timer.cancel()
            self._entry_goal_timer = None
        self.get_logger().info("[GOAL] 发送入口目标点")
        self.send_goal(self.waypoints[0][0], self.waypoints[0][1])

    def _back_phase_done(self):
        if self.rotate_timer:
            self.rotate_timer.cancel()
        angle_diff = self.get_angle_to_target(self.waypoints[0][0], self.waypoints[0][1])
        self.get_logger().info(f"[ROTATION] 后退完成，实时夹角={angle_diff:.1f}°")
        if abs(angle_diff) < Config.ANGLE_THRESHOLD:
            self._finish_rotation()
            return
        self._current_cmd = (0.0, self._current_cmd[1])
        self.get_logger().warn(
            f"[ROTATION] 后退完成→保持舵机停线性 | angular={self._current_cmd[1]:.2f} | 停{Config.ROTATION_PAUSE_DURATION}s")
        self.rotate_timer = self.create_timer(Config.ROTATION_PAUSE_DURATION, self._forward_after_back)

    def _forward_after_back(self):
        if self.rotate_timer:
            self.rotate_timer.cancel()
        if self.rotate_direction == 'right':
            self._current_cmd = (Config.ROTATION_FWD_SPEED, Config.ROTATION_FWD_STEERING)
        else:
            self._current_cmd = (Config.ROTATION_FWD_SPEED, -Config.ROTATION_FWD_STEERING)
        self._rotation_pub_count = 0
        fwd_duration = getattr(self, '_current_fwd_duration', Config.ROTATION_FWD_DURATION)
        self.get_logger().warn(
            f"[ROTATION] 开始前进 | linear={self._current_cmd[0]:.2f}, angular={self._current_cmd[1]:.2f} | "
            f"动态时长={fwd_duration:.2f}s")
        self.rotate_timer = self.create_timer(fwd_duration, self._forward_phase_done)

    def _forward_phase_done(self):
        if self.rotate_timer:
            self.rotate_timer.cancel()
        angle_diff = self.get_angle_to_target(self.waypoints[0][0], self.waypoints[0][1])
        self.get_logger().info(f"[ROTATION] 前进完成，实时夹角={angle_diff:.1f}°")
        if abs(angle_diff) < Config.ANGLE_THRESHOLD:
            self._finish_rotation()
            return
        self._current_cmd = (0.0, self._current_cmd[1])
        self.get_logger().warn(
            f"[ROTATION] 前进完成→保持舵机停线性 | angular={self._current_cmd[1]:.2f} | 停{Config.ROTATION_PAUSE_DURATION}s")
        self.rotate_timer = self.create_timer(Config.ROTATION_PAUSE_DURATION, self._rotation_cycle_done)

    def _rotation_cycle_done(self):
        if self.rotate_timer:
            self.rotate_timer.cancel()
        self.get_logger().info("[ROTATION] 一个节拍完成，进入下一轮")
        self._rotate_once()

    def _start_drive_timer_with_cone_check(self):
        """启动持续发布命令的定时器，同时检测前方锥桶"""
        if self.drive_timer:
            self.drive_timer.cancel()
        self.drive_timer = self.create_timer(Config.CMD_PUBLISH_PERIOD, self._publish_cmd_with_cone_check)

    def _publish_cmd_with_cone_check(self):
        if self._check_cone_in_front():
            self.get_logger().warn("[ROTATION] 🚧 前方检测到锥桶，打断圆弧调整")
            self._cleanup_rotation_timers()
            self.stop_robot()
            self._rotation_interrupted = True
            self.start_smart_backward(reason="圆弧中锥桶打断")
            return

        angle_diff = self.get_angle_to_target(self.waypoints[0][0], self.waypoints[0][1])
        if abs(angle_diff) < Config.ANGLE_THRESHOLD:
            self.get_logger().warn(f"[ROTATION] 实时夹角={angle_diff:.1f}° 达标，立即退出")
            self._finish_rotation()
            return

        self._rotation_pub_count = getattr(self, '_rotation_pub_count', 0) + 1
        if self._rotation_pub_count % 10 == 1:
            self.get_logger().info(
                f"[ROTATION] #{self._rotation_pub_count} 夹角={angle_diff:.1f}° 发送: linear={self._current_cmd[0]:.2f}, angular={self._current_cmd[1]:.2f}")
        self.publish_drive(self._current_cmd[0], self._current_cmd[1])

    def _cleanup_rotation_timers(self):
        if self.rotate_timer:
            self.rotate_timer.cancel()
            self.rotate_timer = None
        if self.drive_timer:
            self.drive_timer.cancel()
            self.drive_timer = None

    # ==================== 静止检测 ====================
    def check_stuck(self):
        if self.blue_hall_done or self.exit_flag:
            return
        if self.sub_state is not None and self.sub_state != 'WAITING_QR':
            return
        if self.state not in ('GO_TO_PRE_QR', 'GO_TO_QR', 'GO_TO_ENTRY'):
            return
        if not self.goal_sent and self.sub_state != 'WAITING_QR':
            return

        # ★ 启动宽限期：目标发送后1秒内不检测静止
        if self.sub_state != 'WAITING_QR' and self.goal_sent_time is not None:
            since_goal = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
            if since_goal < 1.0:
                return

        pose = self.get_current_pose()
        if pose[0] is None:
            return

        x, y, yaw = pose

        if self.last_stuck_position is None:
            self.last_stuck_position = (x, y)
            self.last_stuck_yaw = yaw
            self.last_stuck_time = self.get_clock().now()
            return

        dx = x - self.last_stuck_position[0]
        dy = y - self.last_stuck_position[1]
        moved = math.hypot(dx, dy)
        yaw_diff = abs(yaw - self.last_stuck_yaw)
        yaw_diff = min(yaw_diff, 2 * math.pi - yaw_diff)
        elapsed = (self.get_clock().now() - self.last_stuck_time).nanoseconds / 1e9

        # 位置移动>2cm 或 旋转>5° → 不算静止
        if moved > 0.02 or yaw_diff > 0.08:
            self.last_stuck_position = (x, y)
            self.last_stuck_yaw = yaw
            self.last_stuck_time = self.get_clock().now()
            return

        if elapsed > Config.STUCK_TIMEOUT:
            # QR等待状态额外等1秒给检测机会
            if self.sub_state == 'WAITING_QR':
                qr_elapsed = (self.get_clock().now() - self._qr_wait_start).nanoseconds / 1e9
                if qr_elapsed < 1.0:
                    return
                self.get_logger().warn(f"[STUCK] QR等待{qr_elapsed:.1f}s超时，触发智能移动")

            self.stuck_attempt_count += 1
            self.last_stuck_position = (x, y)
            self.last_stuck_time = self.get_clock().now()

            back_space = self._get_back_space()
            front_space = self._get_front_space()

            if back_space > 0.5:
                self.start_smart_backward(reason=f"静止{self.stuck_attempt_count}次")
            elif front_space > 0.5:
                self.start_smart_forward(reason=f"静止{self.stuck_attempt_count}次")
            else:
                self.get_logger().warn("[STUCK] ❌ 前后都没有空间")


def main():
    rclpy.init()
    node = BlueHall()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()