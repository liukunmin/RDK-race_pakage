#!/usr/bin/env python3
import sys
sys.path.insert(0, '/usr/lib/python3/dist-packages')
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import cv2
import numpy as np
from tf2_ros import TransformListener, Buffer
from config import Config
from smart_movement import SmartMovementMixin
from display_utils import draw_display, set_fullscreen


class YellowChannel(Node, SmartMovementMixin):
    def __init__(self, waypoints, direction, qr_result="", nav_client=None, auto_start=True):
        super().__init__('yellow_channel')
        self.delayed_timer = None
        self.sign_trigger = None
        self.sign_trigger_pending = False

        self.waypoints = waypoints
        self.direction = direction
        self.current_wp_index = 0
        self.task_completed = False
        self.qr_result = qr_result
        self.sign_result_text = ""

        self.get_logger().info("=" * 60)
        self.get_logger().info("🟡 黄色通道模块启动（NavigateToPose + 50Hz决策）")
        self.get_logger().info(f"📍 路径点数量: {len(waypoints)}")
        self.get_logger().info(f"🧭 方向: {direction}")
        self.get_logger().info("=" * 60)

        # ===== 导航客户端（复用blue的连接，避免DDS重新发现） =====
        if nav_client is not None:
            self.nav_client = nav_client
            self.get_logger().info("✅ 复用blue_hall的导航客户端")
        else:
            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ===== costmap清除服务 =====
        self._clear_local = self.create_client(ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')
        self._clear_global = self.create_client(ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')

        # ===== 激光雷达 =====
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # ===== scan中继：转弯时停掉ICP =====
        self.scan_loc_pub = self.create_publisher(LaserScan, '/scan_loc', 10)
        self._last_relay_yaw = None
        self._icp_paused = False

        # ===== 摄像头 =====
        qos_img = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.sub_image = self.create_subscription(CompressedImage, '/image', self._image_callback, qos_img)
        self._latest_image = None

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

        # ===== 状态变量 =====
        self.state = 'NAVIGATING'
        self.goal_handle = None
        self.goal_sent = False
        self.goal_sent_time = None
        self._photo_pending = False
        self._photo_timer = None

        # ===== 智能移动 =====
        self.is_moving = False
        self.move_speed = 0.0
        self.move_steering = 0.0
        self.move_max_dist = 0.0
        self.move_start_time = None
        self.move_monitor_angles = []
        self.move_count = 0
        self.move_pub_timer = None
        self.move_check_timer = None
        self.move_timer = None
        self._rotation_interrupted = False
        self._trying_opposite = False

        # ===== 黄色通道状态 =====
        self.in_yellow_channel = False

        # ===== 静止检测 =====
        self.last_position = None
        self.last_position_time = None
        self.last_yaw = 0.0
        self._last_forward_time = None
        self.stuck_attempt_count = 0
        self.goal_timeout_count = 0
        self.smart_move_count = 0

        # ===== 暂停机制 =====
        self.paused = False

        # ===== 定时器 =====
        self._decision_count = 0
        self.decision_timer = self.create_timer(0.02, self._decision_loop)
        self.stuck_timer = self.create_timer(0.2, self.check_stuck)
        self.goal_timeout_timer = self.create_timer(1.0, self.check_goal_timeout)

        # ===== 屏幕显示 =====
        self.sub_sign_result = self.create_subscription(String, '/sign_result', self.sign_result_callback, 10)
        cv2.namedWindow('QR', cv2.WINDOW_NORMAL)
        self._fullscreen_set = False
        self.display_timer = self.create_timer(0.1, self.update_display)
        self._sign_spin_timer = self.create_timer(0.05, self._spin_sign_trigger)

        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("等待 Nav2...")
        self.get_logger().info("✅ Nav2 服务已就绪")

        self._started = False
        if auto_start:
            self.start_navigation()

    def start_navigation(self):
        """开始导航：自动检测起点 + 发送第一个目标"""
        if self._started:
            return
        self._started = True
        self._auto_set_start_waypoint()
        self.send_current_goal()

    def _auto_set_start_waypoint(self):
        """检测当前位置，跳过已经过的路径点"""
        pose = self.get_current_pose()
        if pose[0] is None:
            self.get_logger().info("[AUTO] 无法获取当前位置，从路径点0开始")
            return

        x, y, _ = pose
        self.get_logger().info(f"[AUTO] 当前位置: ({x:.2f}, {y:.2f})")

        best_idx = 0
        best_dist = float('inf')
        for i, (wx, wy) in enumerate(self.waypoints):
            d = math.hypot(x - wx, y - wy)
            if d < best_dist:
                best_dist = d
                best_idx = i

        if best_idx > 0 and best_dist < 2.0:
            self.current_wp_index = best_idx
            self.get_logger().warn(f"[AUTO] 最近路径点{best_idx}（距离{best_dist:.2f}m），从点{best_idx}开始")
        else:
            self.get_logger().info(f"[AUTO] 从路径点0开始（最近点{best_idx}距离{best_dist:.2f}m）")

    def _image_callback(self, msg):
        self._latest_image = msg
        if self.sign_trigger:
            self.sign_trigger.latest_image = msg
            self.sign_trigger.image_received = True

    # ==================== 大模型触发节点管理 ====================
    def start_sign_trigger(self):
        try:
            from sign_trigger import SignTrigger
            self.sign_trigger = SignTrigger(waypoints=self.waypoints)
            self.get_logger().info("📸 ✅ 大模型触发节点已启动")
        except Exception as e:
            self.get_logger().error(f"📸 ❌ 启动大模型触发节点失败: {e}")

    def stop_sign_trigger(self):
        if self.sign_trigger:
            try:
                self.sign_trigger.destroy_node()
                self.sign_trigger = None
                self.get_logger().info("📸 ✅ 大模型触发节点已关闭")
            except Exception as e:
                self.get_logger().warn(f"📸 关闭大模型触发节点时出错: {e}")

    # ==================== 屏幕显示 ====================
    def _spin_sign_trigger(self):
        if self.sign_trigger:
            rclpy.spin_once(self.sign_trigger, timeout_sec=0.01)
            if self.sign_trigger_pending and not self.sign_trigger.triggered:
                if self.sign_trigger.image_received and self.sign_trigger.latest_image is not None:
                    self.get_logger().warn("📸 sign_trigger 已收到图像，触发拍照！")
                    self.sign_trigger._trigger_recognition()
                    self.sign_trigger_pending = False

    def sign_result_callback(self, msg):
        self.sign_result_text = msg.data
        self.get_logger().warn(f"[SIGN] 识别结果: {self.sign_result_text}")

    def update_display(self):
        try:
            direction_cn = "顺时针" if self.direction == "clockwise" else "逆时针"
            display = draw_display(self.qr_result, self.sign_result_text, True, direction_cn)
            display = cv2.rotate(display, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imshow('QR', display)
            set_fullscreen('QR')
            cv2.waitKey(1)
        except Exception:
            pass

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
        self._relay_scan(msg)

    def _relay_scan(self, msg):
        pose = self.get_current_pose()
        if pose[0] is None:
            self.scan_loc_pub.publish(msg)
            return
        yaw = pose[2]
        if self._last_relay_yaw is not None:
            yaw_diff = abs(yaw - self._last_relay_yaw)
            yaw_diff = min(yaw_diff, 2 * math.pi - yaw_diff)
            if yaw_diff > 0.05:
                if not self._icp_paused:
                    self.get_logger().warn(f"[ICP] 检测到旋转(yaw变化{math.degrees(yaw_diff):.1f}°)，暂停ICP")
                    self._icp_paused = True
            else:
                if self._icp_paused:
                    self.get_logger().info("[ICP] 旋转结束，恢复ICP")
                    self._icp_paused = False
        self._last_relay_yaw = yaw
        if not self._icp_paused:
            self.scan_loc_pub.publish(msg)

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

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

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

    # ==================== NavigateToPose 导航 ====================
    def send_current_goal(self):
        """发送当前路径点的NavigateToPose目标"""
        if self.task_completed:
            return
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().warn("🏁 所有路径点完成！任务结束")
            self.task_completed = True
            self.state = 'DONE'
            self.stop_robot()
            return

        if self.goal_sent:
            return

        x, y = self.waypoints[self.current_wp_index]
        self.get_logger().info(f"[GOAL] 点[{self.current_wp_index}]: ({x:.2f}, {y:.2f})")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0

        self.goal_sent = True
        self.goal_sent_time = self.get_clock().now()
        self.goal_timeout_count = 0
        self.smart_move_count = 0

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.goal_handle = goal_handle
                self.get_logger().info(f"[GOAL] ✅ 点[{self.current_wp_index}]目标已接受")
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self._goal_result_callback)
            else:
                self.get_logger().error(f"[GOAL] ❌ 点[{self.current_wp_index}]目标被拒绝")
                self.goal_sent = False
                self.current_wp_index += 1
                self.send_current_goal()
        except Exception as e:
            self.get_logger().error(f"[GOAL] 异常: {e}")
            self.goal_sent = False

    def _goal_result_callback(self, future):
        """导航目标完成回调"""
        try:
            result = future.result()
            status = result.status
            self.get_logger().info(f"[GOAL] 点[{self.current_wp_index}]完成 status={status}")
        except Exception as e:
            self.get_logger().error(f"[GOAL] 结果异常: {e}")

        if self.is_moving:
            self.get_logger().info("[GOAL] 被中断（smart movement接管）")
            return

        if self.task_completed:
            return

        self._check_arrival_and_advance()

    def _check_arrival_and_advance(self):
        """检查是否到达当前路径点，到达则切换到下一个"""
        if self.task_completed:
            return
        if self.current_wp_index >= len(self.waypoints):
            return

        pose = self.get_current_pose()
        if pose[0] is None:
            return

        tx, ty = self.waypoints[self.current_wp_index]
        dist = math.hypot(pose[0] - tx, pose[1] - ty)
        tol = Config.WAYPOINT_TOLERANCES.get(self.current_wp_index, 0.5)

        if dist < tol:
            self._on_arrival()

    # ==================== 50Hz 独立决策循环 ====================
    def _decision_loop(self):
        if self.task_completed or self.current_wp_index >= len(self.waypoints):
            return
        if not self.goal_sent or self._photo_pending:
            return
        if self.is_moving:
            return

        self._decision_count += 1
        self._check_arrival_and_advance()

    def _on_arrival(self):
        """到达当前路径点"""
        self.get_logger().warn(f"[ARRIVAL] ✅ 到达点 {self.current_wp_index}")
        self.goal_timeout_count = 0
        self.smart_move_count = 0

        # 路径点2：启动 sign_trigger
        if self.current_wp_index == 2 and self.sign_trigger is None:
            self.in_yellow_channel = True
            self.get_logger().warn("🚪 进入黄色通道，启动大模型触发节点")
            self.start_sign_trigger()

        # 路径点3：触发拍照
        if self.current_wp_index == 3:
            self.get_logger().warn("📸 到达立牌识别点，触发拍照...")
            self.sign_trigger_pending = True
            self._photo_pending = True
            self._photo_timer = self.create_timer(3.0, self._after_photo)
            return

        # 最终点
        if self.current_wp_index >= len(self.waypoints) - 1:
            self.get_logger().warn("🏁 任务完成！")
            self.cancel_current_goal()
            self.stop_robot()
            self.task_completed = True
            self.state = 'DONE'
            return

        # 不cancel直接发下一个目标，避免停车
        self.goal_handle = None
        self.goal_sent = False
        self.current_wp_index += 1
        self.send_current_goal()

    def _after_photo(self):
        """拍照超时后继续导航"""
        if self._photo_timer:
            self._photo_timer.cancel()
            self._photo_timer = None
        self._photo_pending = False
        # ★ 停止sign_trigger，防止它继续触发拍照干扰导航
        self.stop_sign_trigger()
        self.sign_trigger_pending = False
        self.get_logger().info("📸 拍照阶段结束，继续导航")

        if self.current_wp_index >= len(self.waypoints) - 1:
            self.get_logger().warn("🏁 没有更多路径点，任务结束")
            self.task_completed = True
            self.state = 'DONE'
            self.stop_robot()
            return

        self.goal_handle = None
        self.goal_sent = False
        self.current_wp_index += 1
        self.send_current_goal()

    # ==================== 静止检测 ====================
    def check_stuck(self):
        if self.paused or self.task_completed or self.state != 'NAVIGATING':
            return
        if not self.goal_sent or self._photo_pending:
            return
        if self.is_moving:
            return

        # 启动宽限期：目标发送后1秒内不检测
        if self.goal_sent_time is not None:
            since_goal = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
            if since_goal < 1.0:
                return

        pose = self.get_current_pose()
        if pose[0] is None:
            return

        x, y, yaw = pose

        if self.last_position is None:
            self.last_position = (x, y)
            self.last_yaw = yaw
            self.last_position_time = self.get_clock().now()
            self._last_forward_time = self.get_clock().now()
            return

        dx = x - self.last_position[0]
        dy = y - self.last_position[1]
        moved = math.hypot(dx, dy)
        yaw_diff = abs(yaw - self.last_yaw)
        yaw_diff = min(yaw_diff, 2 * math.pi - yaw_diff)
        elapsed = (self.get_clock().now() - self.last_position_time).nanoseconds / 1e9
        since_forward = (self.get_clock().now() - self._last_forward_time).nanoseconds / 1e9

        # 前进了 → 重置所有计时器
        if moved > 0.02:
            self.last_position = (x, y)
            self.last_yaw = yaw
            self.last_position_time = self.get_clock().now()
            self._last_forward_time = self.get_clock().now()
            return

        # 只旋转没前进 → 更新yaw但不重置前进计时
        if yaw_diff > 0.08:
            self.last_yaw = yaw
            self.last_position_time = self.get_clock().now()
            # 原地旋转超过3秒没前进 → 卡住
            if since_forward > 3.0:
                self.get_logger().warn(f"[STUCK] 原地旋转{since_forward:.1f}s无前进，触发智能移动")
                self._trigger_smart_move()
            return

        # 完全静止
        if elapsed > Config.STUCK_TIMEOUT:
            self.get_logger().warn(f"[STUCK] 静止{elapsed:.1f}s，触发智能移动")
            self._trigger_smart_move()

    def _trigger_smart_move(self):
        """触发智能移动（后退/前进）+ 清除costmap"""
        self.stuck_attempt_count += 1
        self.smart_move_count += 1
        self.last_position = None
        self.last_position_time = None
        self._last_forward_time = self.get_clock().now()

        # ★ 清除costmap中的幻影障碍物
        if self._clear_local.service_is_ready():
            self._clear_local.call_async(ClearEntireCostmap.Request())
        if self._clear_global.service_is_ready():
            self._clear_global.call_async(ClearEntireCostmap.Request())
        self.get_logger().warn(f"[STUCK] 已清除local+global costmap，触发智能移动(第{self.stuck_attempt_count}次)")

        back_space = self._get_back_space()
        front_space = self._get_front_space()
        if back_space > 0.5:
            self.start_smart_backward(reason=f"卡住{self.stuck_attempt_count}次")
        elif front_space > 0.5:
            self.start_smart_forward(reason=f"卡住{self.stuck_attempt_count}次")
        else:
            self.get_logger().warn("[STUCK] ❌ 前后都没有空间")

    # ==================== 目标超时 ====================
    def check_goal_timeout(self):
        if self.paused or not self.goal_sent or self.state != 'NAVIGATING':
            return
        if self._photo_pending:
            return
        if self.is_moving:
            return

        elapsed = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
        if elapsed > Config.GOAL_TIMEOUT:
            self.goal_timeout_count += 1
            self.get_logger().warn(f"[TIMEOUT] 点{self.current_wp_index}超时(第{self.goal_timeout_count}次)")
            if self.goal_timeout_count >= 2:
                self.goal_timeout_count = 0
                self.smart_move_count += 1
                self.get_logger().warn(f"[TIMEOUT] 连续超时，触发智能移动(第{self.smart_move_count}次)")
                self.cancel_current_goal()
                back_space = self._get_back_space()
                front_space = self._get_front_space()
                if back_space > 0.3:
                    self.start_smart_backward(reason="连续超时")
                elif front_space > 0.3:
                    self.start_smart_forward(reason="连续超时")
                else:
                    self.get_logger().warn("[TIMEOUT] ❌ 前后无空间")


def main():
    rclpy.init()
    node = YellowChannel([], "counter_clockwise")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
