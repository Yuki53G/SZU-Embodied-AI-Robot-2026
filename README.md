# 第三届深圳大学具身智能机器人大赛 - 医路先锋队
🏆 **一等奖** | 2026年6月

## 项目简介
基于OriginBot双轮自平衡机器人的具身智能比赛项目，完成自主导航、二维码识别、路径规划等比赛任务。

## 技术栈
- **机器人平台**：OriginBot双轮自平衡机器人
- **操作系统**：ROS2 Humble (TogetheROS)
- **导航框架**：Nav2导航栈
- **视觉识别**：二维码识别
- **开发环境**：Ubuntu Linux
- **编程语言**：Python / C

## 系统启动流程
### 1. 底盘+雷达+IMU启动
source /opt/tros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch originbot_bringup originbot.launch.py use_lidar:=true use_imu:=true
### 2. 相机启动
cd /userdata/dev_ws
ros2 launch originbot_bringup camera_websoket_display.launch.py
### 3. 二维码识别
ros2 run originbot_qrcode_detect qr_decoder
### 4. Nav2 导航启动
ros2 launch originbot_nav2_competition nav2_competition.launch.py \
  map:=/root/ros2_ws/maps/originbot_competition_map.yaml
### 5. 初始位姿发布
ros2 run originbot_nav2_competition publish_initial_pose
# 发布位姿：x=-0.373, y=-0.222, yaw=31.2°
### 6. 任务节点启动
ros2 run originbot_nav2_competition nav2_mission_node

# 团队成员
颜智琳、邹希桐、王鑫
