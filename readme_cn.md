\# OriginCar 智能车竞赛系统



基于 ROS2 Humble 的智能车竞赛导航与控制软件栈，用于第二十一届全国大学生智能汽车竞赛。



\## 系统架构

┌─────────────────────────────────────────────────┐

│ race\_package │

│ main.py → BlueHall → YellowChannel │

│ (二维码检测 → 路径规划 → 立牌识别) │

└─────────────────────┬───────────────────────────┘

│

┌─────────────────────▼───────────────────────────┐

│ origin\_navigation2 │

│ Nav2 配置 (定位/规划/控制) │

│ lidar\_loc (纯雷达定位，替代 AMCL) │

└─────────────────────┬───────────────────────────┘

│

┌─────────────────────▼───────────────────────────┐

│ origincar │

│ 底盘驱动 (STM32 串口通信) │

│ /odom 发布 /cmd\_vel 订阅 │

└─────────────────────┬───────────────────────────┘

│

┌─────────────────────▼───────────────────────────┐

│ lslidar\_ros2 │

│ 激光雷达驱动 (N10 串口) │

└─────────────────────────────────────────────────┘





\## 目录结构

/origin\_ws/

├── src/

│ ├── lidar\_loc/ # 纯雷达定位

│ ├── lslidar\_ros2/ # 激光雷达驱动

│ ├── origincar/ # 底盘驱动

│ │ ├── origincar\_base/ # 串口通信

│ │ ├── origincar\_bringup/ # 启动文件

│ │ └── origincar\_description/ # URDF

│ ├── origin\_navigation2/ # Nav2 配置

│ └── race\_package/ # 比赛任务

│ ├── main.py # 入口

│ ├── blue\_hall.py # 蓝色大厅

│ ├── yellow\_channel.py # 黄色通道

│ ├── smart\_movement.py # 智能避障

│ └── config.py # 参数配置

├── scripts/

│ └── emergency\_stop.sh # 紧急停止

└── install/ build/ log/





\## 快速启动



\### 1. 编译

cd /origin\_ws

colcon build --symlink-install

source install/setup.bash





2\. 启动硬件



\# 终端1：底盘

ros2 launch origincar\_bringup origincar\_bringup.launch.py



\# 终端2：激光雷达

ros2 launch lslidar\_driver lsn10\_launch.py



\# 终端3：导航

ros2 launch origin\_navigation2 navigation2.launch.py



3\. 启动比赛任务

ros2 run race\_package main





核心参数

参数	位置	说明

导航参数	origin\_navigation2/config/nav2\_params.yaml	速度/容忍度/控制器

任务参数	race\_package/config.py	坐标点/路径/触发距离

刹车参数	origincar\_base/src/origincar\_base.cpp	BRAKE\_SPEED/BRAKE\_DURATION



紧急停止

cd /origin\_ws

./emergency\_stop.sh



依赖

Ubuntu 22.04

ROS2 Humble

Nav2

OpenCV

微信二维码检测 (可选)



许可证

Apache-2.0

