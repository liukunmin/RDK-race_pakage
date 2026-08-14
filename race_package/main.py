#!/usr/bin/env python3
import os
os.environ.setdefault('DISPLAY', ':0')
os.environ.setdefault('XAUTHORITY', '/home/sunrise/.Xauthority')

import sys
sys.path.insert(0, '/usr/lib/python3/dist-packages')
import rclpy
import cv2
from blue_hall import BlueHall
from yellow_channel import YellowChannel


def main():
    print("=" * 60)
    print("🤖 智能汽车竞赛主程序")
    print("=" * 60)
    print("🔵 开始蓝色大厅阶段...")

    rclpy.init()

    blue = BlueHall()

    # 旋转直到第一个目标被接受
    while rclpy.ok() and not blue.first_goal_accepted:
        rclpy.spin_once(blue, timeout_sec=0.05)

    # 立即取消目标，让车不动
    if blue.goal_handle:
        blue.goal_handle.cancel_goal_async()
        blue.goal_handle = None
    blue.goal_sent = False
    blue.stop_robot()

    # 等待回车键（由run_all.sh的read触发，这里也等一次）
    print("\n" + "=" * 60)
    print("⏸️  按回车键启动小车...")
    print("=" * 60)
    input()

    # 按回车后立即重发目标，车开跑
    blue.send_current_goal()
    print("\n▶️  启动！\n")

    # 继续蓝色大厅
    yellow = None
    while rclpy.ok() and not blue.exit_flag:
        rclpy.spin_once(blue, timeout_sec=0.05)
        # ★ 检测到二维码后预创建YellowChannel（圆弧调整期间完成初始化）
        if yellow is None and blue.qr_detected and blue.waypoints:
            yellow = YellowChannel(blue.waypoints, blue.direction, blue.qr_last_result,
                                   nav_client=blue.nav_client, auto_start=False)
            print("🟡 黄色通道预加载完成（圆弧调整期间初始化）")

    waypoints = blue.waypoints
    direction = blue.direction
    qr_result = blue.qr_last_result

    print("=" * 60)
    print(f"🔵 蓝色大厅完成，exit_flag={blue.exit_flag}")
    print(f"📌 路径点共 {len(waypoints)} 个，方向: {direction}")
    print(f"🔢 二维码数字: {qr_result}")
    print("=" * 60)

    if not waypoints:
        print("❌ 蓝色大厅未能加载路径点，退出")
        blue.destroy_node()
        rclpy.shutdown()
        return

    print("🟡 开始黄色通道阶段...")
    if yellow is not None:
        # ★ 预加载已完成，直接启动导航（零延迟）
        yellow.start_navigation()
    else:
        # 兜底：如果未能预加载，正常创建
        yellow = YellowChannel(waypoints, direction, qr_result, nav_client=blue.nav_client)
    blue.destroy_node()
    rclpy.spin(yellow)
    yellow.destroy_node()

    cv2.destroyAllWindows()

    print("=" * 60)
    print("✅ 全部任务完成！")
    print("=" * 60)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
