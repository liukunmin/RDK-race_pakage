#!/usr/bin/env python3
"""
laser_corrector.py - 激光数据校正节点
订阅 /raw_scan，校正后发布 /scan
校正内容：角度偏移补偿、距离偏移/缩放、噪点滤波、距离范围过滤

启动:
    python3 src/race_package/scripts/laser_corrector.py

动态调整参数:
    ros2 param set /laser_corrector angle_offset 0.05
    ros2 param set /laser_corrector distance_offset -0.02
    ros2 param set /laser_corrector distance_scale 1.01
    ros2 param set /laser_corrector enable_noise_filter true
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LaserCorrector(Node):
    def __init__(self):
        super().__init__('laser_corrector')

        self.angle_offset = self.declare_parameter('angle_offset', 0.0).value
        self.distance_offset = self.declare_parameter('distance_offset', 0.0).value
        self.distance_scale = self.declare_parameter('distance_scale', 1.0).value
        self.min_distance = self.declare_parameter('min_distance', 0.15).value
        self.max_distance = self.declare_parameter('max_distance', 50.0).value
        self.enable_noise_filter = self.declare_parameter('enable_noise_filter', True).value

        self.sub = self.create_subscription(LaserScan, '/raw_scan', self.scan_callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        self.get_logger().info("=" * 60)
        self.get_logger().info("激光数据校正节点已启动")
        self.get_logger().info(f"  角度偏移: {self.angle_offset} rad")
        self.get_logger().info(f"  距离偏移: {self.distance_offset} m")
        self.get_logger().info(f"  距离缩放: {self.distance_scale}")
        self.get_logger().info(f"  距离范围: [{self.min_distance}, {self.max_distance}] m")
        self.get_logger().info(f"  噪点滤波: {'开启' if self.enable_noise_filter else '关闭'}")
        self.get_logger().info("  /raw_scan -> /scan")
        self.get_logger().info("=" * 60)

    def scan_callback(self, msg):
        corrected = LaserScan()
        corrected.header = msg.header
        corrected.header.frame_id = msg.header.frame_id

        corrected.angle_min = msg.angle_min + self.angle_offset
        corrected.angle_max = msg.angle_max + self.angle_offset
        corrected.angle_increment = msg.angle_increment
        corrected.time_increment = msg.time_increment
        corrected.scan_time = msg.scan_time
        corrected.range_min = max(msg.range_min, self.min_distance)
        corrected.range_max = min(msg.range_max, self.max_distance)

        ranges = list(msg.ranges)
        filtered = [0.0] * len(ranges)

        for i in range(len(ranges)):
            r = ranges[i]
            if r == float('inf') or r == float('nan') or r <= 0.0:
                filtered[i] = float('inf')
                continue
            r_corrected = r * self.distance_scale + self.distance_offset
            if r_corrected < self.min_distance or r_corrected > self.max_distance:
                filtered[i] = float('inf')
                continue
            filtered[i] = r_corrected

        if self.enable_noise_filter and len(filtered) > 2:
            for i in range(1, len(filtered) - 1):
                curr = filtered[i]
                prev = filtered[i - 1]
                nxt = filtered[i + 1]
                if curr == float('inf') or prev == float('inf') or nxt == float('inf'):
                    continue
                threshold = 0.015 + 0.001 * curr
                diff_prev = abs(curr - prev) / max(prev, 0.01)
                diff_next = abs(curr - nxt) / max(nxt, 0.01)
                if diff_prev > threshold and diff_next > threshold:
                    filtered[i] = float('inf')

        corrected.ranges = filtered
        corrected.intensities = msg.intensities

        self.pub.publish(corrected)


def main(args=None):
    rclpy.init(args=args)
    node = LaserCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
