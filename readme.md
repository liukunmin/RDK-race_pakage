Apache-2.0



\# OriginCar Autonomous Racing System



ROS2 Humble-based autonomous racing stack for the 21st National College Smart Car Competition.



\## System Architecture

┌─────────────────────────────────────────────────┐

│ race\_package │

│ main.py → BlueHall → YellowChannel │

│ (QR detection → Path planning → Sign recon) │

└─────────────────────┬───────────────────────────┘

│

┌─────────────────────▼───────────────────────────┐

│ origin\_navigation2 │

│ Nav2 config (localization/planning/control) │

│ lidar\_loc (radar-only localization) │

└─────────────────────┬───────────────────────────┘

│

┌─────────────────────▼───────────────────────────┐

│ origincar │

│ Chassis driver (STM32 serial communication) │

│ /odom pub / /cmd\_vel sub │

└─────────────────────┬───────────────────────────┘

│

┌─────────────────────▼───────────────────────────┐

│ lslidar\_ros2 │

│ LiDAR driver (N10 serial) │

└─────────────────────────────────────────────────┘



text



\## Directory Structure

/origin\_ws/

├── src/

│ ├── lidar\_loc/ # Radar-only localization

│ ├── lslidar\_ros2/ # LiDAR driver

│ ├── origincar/ # Chassis driver

│ │ ├── origincar\_base/ # Serial communication

│ │ ├── origincar\_bringup/ # Launch files

│ │ └── origincar\_description/ # URDF

│ ├── origin\_navigation2/ # Nav2 config

│ └── race\_package/ # Race tasks

│ ├── main.py # Entry

│ ├── blue\_hall.py # Blue hall

│ ├── yellow\_channel.py # Yellow channel

│ ├── smart\_movement.py # Obstacle avoidance

│ └── config.py # Parameters

├── scripts/

│ └── emergency\_stop.sh # Emergency stop

└── install/ build/ log/



text



\## Quick Start



\### 1. Build



```bash

cd /origin\_ws

colcon build --symlink-install

source install/setup.bash

2\. Start Hardware

bash

\# Terminal 1: Chassis

ros2 launch origincar\_bringup origincar\_bringup.launch.py



\# Terminal 2: LiDAR

ros2 launch lslidar\_driver lsn10\_launch.py



\# Terminal 3: Navigation

ros2 launch origin\_navigation2 navigation2.launch.py

3\. Start Race Task

bash

ros2 run race\_package main

Key Parameters

Parameter	Location	Description

Navigation	origin\_navigation2/config/nav2\_params.yaml	Velocity/Tolerance/Controller

Task	race\_package/config.py	Waypoints/Trigger distance

Braking	origincar\_base/src/origincar\_base.cpp	BRAKE\_SPEED/BRAKE\_DURATION

Emergency Stop

bash

cd /origin\_ws

./emergency\_stop.sh

Dependencies

Ubuntu 22.04



ROS2 Humble



Nav2



OpenCV



WeChat QR code detection (optional)



License

Apache-2.0

