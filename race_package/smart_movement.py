#!/usr/bin/env python3
import math
from config import Config


class SmartMovementMixin:
    """智能移动混合类（纯避障，无加速）"""

    def _get_front_space(self):
        if self.lidar_ranges is None:
            return 999.0
        min_dist = 999.0
        for angle_offset in range(Config.FRONT_SCAN_LOW, Config.FRONT_SCAN_HIGH + 1, Config.SCAN_ANGLE_STEP):
            angle_rad = math.radians(angle_offset)
            idx = int((angle_rad - self.lidar_angle_min) / self.lidar_angle_increment)
            if 0 <= idx < len(self.lidar_ranges):
                dist = self.lidar_ranges[idx]
                if Config.MIN_VALID_DIST < dist < min_dist:
                    min_dist = dist
        return min_dist

    def _get_back_space(self):
        if self.lidar_ranges is None:
            return 999.0
        min_dist = 999.0
        for angle_offset in range(Config.FRONT_SCAN_LOW, Config.FRONT_SCAN_HIGH + 1, Config.SCAN_ANGLE_STEP):
            angle_rad = math.radians(180 + angle_offset)
            idx = int((angle_rad - self.lidar_angle_min) / self.lidar_angle_increment)
            if 0 <= idx < len(self.lidar_ranges):
                dist = self.lidar_ranges[idx]
                if Config.MIN_VALID_DIST < dist < min_dist:
                    min_dist = dist
        return min_dist

    def _check_direction_blocked(self, direction='front', threshold=0.3):
        if self.lidar_ranges is None:
            return False
        for angle_offset in range(Config.FRONT_SCAN_LOW, Config.FRONT_SCAN_HIGH + 1, Config.SCAN_ANGLE_STEP):
            if direction == 'front':
                angle_rad = math.radians(angle_offset)
            else:
                angle_rad = math.radians(180 + angle_offset)
            idx = int((angle_rad - self.lidar_angle_min) / self.lidar_angle_increment)
            if 0 <= idx < len(self.lidar_ranges):
                dist = self.lidar_ranges[idx]
                if Config.MIN_VALID_DIST < dist < threshold:
                    return True
        return False

    def get_back_distance(self, angle):
        return self.back_distances.get(angle, 999.0)

    def get_front_distance(self, angle):
        return self.front_distances.get(angle, 999.0)

    def get_current_back_dist(self):
        if not self.move_monitor_angles:
            return 999.0
        dists = [self.get_back_distance(a) for a in self.move_monitor_angles]
        return min(dists)

    def get_current_front_dist(self):
        if not self.move_monitor_angles:
            return 999.0
        dists = [self.get_front_distance(a) for a in self.move_monitor_angles]
        return min(dists)

    def get_best_backward_steering(self):
        SAFE_DIST = Config.SAFE_DIST
        step = Config.SCAN_ANGLE_STEP

        sectors = {
            "right": (Config.BACK_SECTOR_RIGHT_LOW, Config.BACK_SECTOR_RIGHT_HIGH,
                      Config.BACKWARD_STEERING_RIGHT, "右后退"),
            "straight": (Config.BACK_SECTOR_STRAIGHT_LOW, Config.BACK_SECTOR_STRAIGHT_HIGH,
                         0.0, "直后退"),
            "left": (Config.BACK_SECTOR_LEFT_LOW, Config.BACK_SECTOR_LEFT_HIGH,
                     Config.BACKWARD_STEERING_LEFT, "左后退"),
        }

        for direction in Config.BACKWARD_PRIORITY:
            low, high, steering, desc = sectors[direction]
            safe = True
            for angle in range(low, high + 1, step):
                if self.get_back_distance(angle) < SAFE_DIST:
                    safe = False
                    break
            if safe:
                self.move_monitor_angles = list(range(low, high + 1, step))
                return steering, desc

        return None, "无安全空间"

    def get_best_forward_steering(self):
        SAFE_DIST = Config.SAFE_DIST
        step = Config.SCAN_ANGLE_STEP
        scan_low = Config.FRONT_SCAN_LOW
        scan_high = Config.FRONT_SCAN_HIGH
        center_low = Config.FRONT_CENTER_LOW
        center_high = Config.FRONT_CENTER_HIGH
        
        # 直前进
        center_safe = True
        for angle in range(center_low, center_high + 1, step):
            if self.get_front_distance(angle) < SAFE_DIST:
                center_safe = False
                break
        if center_safe:
            self.move_monitor_angles = list(range(center_low, center_high + 1, step))
            return 0.0, "直前进"
        
        # 左前进
        left_safe = True
        for angle in range(scan_low, center_low, step):
            if self.get_front_distance(angle) < SAFE_DIST:
                left_safe = False
                break
        if left_safe:
            self.move_monitor_angles = list(range(scan_low, center_low, step))
            return Config.FORWARD_STEERING_LEFT, "左前进"
        
        # 右前进
        right_safe = True
        for angle in range(center_high + 1, scan_high + 1, step):
            if self.get_front_distance(angle) < SAFE_DIST:
                right_safe = False
                break
        if right_safe:
            self.move_monitor_angles = list(range(center_high + 1, scan_high + 1, step))
            return Config.FORWARD_STEERING_RIGHT, "右前进"
        
        return None, "无安全空间"

    def start_smart_backward(self, reason=""):
        if getattr(self, 'state', None) in ['BACKING', 'FORWARD']:
            return
        if getattr(self, '_trying_opposite', False):
            self.get_logger().warn("[STUCK] ❌ 前后都没有空间，无法智能移动")
            return
        self._clear_movement_timers()
        self.cancel_current_goal()
        # 直接调用分方向检查：直退>右退>左退，每方向用SAFE_DIST(0.2m)判断
        steering, desc = self.get_best_backward_steering()
        if steering is None:
            self._trying_opposite = True
            self.start_smart_forward(reason=f"{reason}->无方向转前进")
            self._trying_opposite = False
            return
        self._start_movement(
            speed=Config.BACKWARD_SPEED,
            steering=steering,
            reason=reason,
            desc=desc,
            mode='BACKING'
        )

    def start_smart_forward(self, reason=""):
        if getattr(self, 'state', None) in ['BACKING', 'FORWARD']:
            return
        if getattr(self, '_trying_opposite', False):
            self.get_logger().warn("[STUCK] ❌ 前后都没有空间，无法智能移动")
            return
        self._clear_movement_timers()
        self.cancel_current_goal()
        # 直接调用分方向检查：直进>左进>右进，每方向用SAFE_DIST(0.2m)判断
        steering, desc = self.get_best_forward_steering()
        if steering is None:
            self._trying_opposite = True
            self.start_smart_backward(reason=f"{reason}->无方向转后退")
            self._trying_opposite = False
            return
        self._start_movement(
            speed=Config.FORWARD_SPEED,
            steering=steering,
            reason=reason,
            desc=desc,
            mode='FORWARD'
        )

    def _start_movement(self, speed, steering, reason, desc, mode):
        self.move_count = getattr(self, 'move_count', 0) + 1
        self.is_moving = True
        self.move_speed = speed
        self.move_steering = steering
        self._state_before_move = getattr(self, 'state', 'GO_TO_QR')

        if mode == 'BACKING':
            self.move_max_dist = self.get_current_back_dist()
        else:
            self.move_max_dist = self.get_current_front_dist()

        self.move_start_time = self.get_clock().now()
        self.get_logger().info(f"[{mode}] 原因: {reason}, 方向: {desc}")

        setattr(self, 'state', mode)
        self._clear_movement_timers()

        self.move_pub_timer = self.create_timer(Config.CMD_PUBLISH_PERIOD, self._move_publish_loop)

        if mode == 'BACKING':
            self.move_check_timer = self.create_timer(Config.SPACE_CHECK_PERIOD, self._check_backward_space)
        else:
            self.move_check_timer = self.create_timer(Config.SPACE_CHECK_PERIOD, self._check_forward_space)

        self.move_timer = self.create_timer(Config.MAX_MOVE_TIME, self._move_timeout)
        self.publish_drive(self.move_speed, self.move_steering, reason=f"{mode}启动")

    def _move_publish_loop(self):
        state = getattr(self, 'state', None)
        if state not in ['BACKING', 'FORWARD']:
            return

        blocked = False
        if state == 'BACKING':
            if self._check_direction_blocked('back', threshold=Config.SAFE_STOP_DIST):
                blocked = True
        elif state == 'FORWARD':
            if self._check_direction_blocked('front', threshold=Config.SAFE_STOP_DIST):
                blocked = True

        if blocked:
            self.get_logger().info(f"[MOVE] 🚧 {state} 方向检测到障碍物，立即停止")
            self._stop_move()
            return

        self.publish_drive(self.move_speed, self.move_steering)

    def _check_backward_space(self):
        state = getattr(self, 'state', None)
        if state != 'BACKING':
            return

        current_dist = self.get_current_back_dist()
        elapsed = (self.get_clock().now() - self.move_start_time).nanoseconds / 1e9

        if elapsed < Config.MOVE_STABILIZE_TIME:
            self.move_max_dist = max(self.move_max_dist, current_dist)
            return

        if current_dist > self.move_max_dist:
            self.move_max_dist = current_dist
            return

        if current_dist < self.move_max_dist * Config.SPACE_SHRINK_RATIO:
            self.get_logger().info("[BACKWARD] 空间缩小停止")
            self._stop_move()

    def _check_forward_space(self):
        state = getattr(self, 'state', None)
        if state != 'FORWARD':
            return

        current_dist = self.get_current_front_dist()
        elapsed = (self.get_clock().now() - self.move_start_time).nanoseconds / 1e9

        if elapsed < Config.MOVE_STABILIZE_TIME:
            self.move_max_dist = max(self.move_max_dist, current_dist)
            return

        if current_dist > self.move_max_dist:
            self.move_max_dist = current_dist
            return

        if current_dist < self.move_max_dist * Config.SPACE_SHRINK_RATIO:
            self.get_logger().info("[FORWARD] 空间缩小停止")
            self._stop_move()

    def _move_timeout(self):
        state = getattr(self, 'state', None)
        if state in ['BACKING', 'FORWARD']:
            self.get_logger().info("[MOVE] 达到最大移动时间，强制停止")
            self._stop_move()

    def _stop_move(self):
        self._clear_movement_timers()
        self.stop_robot()
        self.is_moving = False
        self.move_monitor_angles = []

        setattr(self, 'state', getattr(self, '_state_before_move', 'GO_TO_QR'))
        self.goal_sent = False
        self.goal_handle = None
        if hasattr(self, 'last_position'):
            self.last_position = None
            self.last_position_time = None
        self.goal_sent_time = self.get_clock().now()

        self.get_logger().info("[MOVE] 移动完成")

        # ★★★ 恢复：智能后退完成后，如果需要进入圆弧调整则进入 ★★★
        if hasattr(self, '_rotation_interrupted') and self._rotation_interrupted:
            self._rotation_interrupted = False
            self.get_logger().info("[STATE] 智能后退完成 → 继续圆弧调整")
            if hasattr(self, '_enter_rotation'):
                self._enter_rotation()
            return

        self.delayed_timer = self.create_timer(Config.DELAYED_GOAL_TIME, self._delayed_send_goal)

    def _delayed_send_goal(self):
        if self.delayed_timer:
            self.delayed_timer.cancel()
            self.delayed_timer = None
        self.send_current_goal()

    def _clear_movement_timers(self):
        for name in ['move_pub_timer', 'move_check_timer', 'move_timer']:
            timer = getattr(self, name, None)
            if timer:
                timer.cancel()
                setattr(self, name, None)
