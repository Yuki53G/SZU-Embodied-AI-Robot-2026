# 第三届深圳大学具身智能机器人大赛 - 医路先锋队
🏆 **一等奖** | 2026年6月

## 项目简介
基于OriginBot双轮自平衡机器人的具身智能医疗辅助系统，集成ROS2 Humble导航栈、激光雷达SLAM、二维码视觉识别、多传感器融合的完整自主导航方案。

## 技术架构
### 硬件平台
- **机器人本体**：OriginBot双轮自平衡机器人
- **主控**：STM32F103C8T6（底层运动控制）
- **感知**：激光雷达、IMU、MPU6050六轴陀螺仪、USB摄像头
- **上位机**：Ubuntu Linux

### 软件栈
- **操作系统**：ROS2 Humble (TogetheROS)
- **导航框架**：Nav2 + AMCL定位
- **路径规划**：Nav2 Planner Server + Controller Server
- **视觉识别**：OpenCV二维码检测与定位
- **控制算法**：PID双轮自平衡算法
- **坐标变换**：TF2坐标变换系统

## 系统启动流程
### 1. 底盘与传感器启动
source /opt/tros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch originbot_bringup originbot.launch.py use_lidar:=true use_imu:=true

### 2. 相机与二维码识别
bash
运行
# 启动相机
ros2 launch originbot_bringup camera_websoket_display.launch.py

# 二维码识别节点
ros2 run originbot_qrcode_detect qr_decoder
### 3. Nav2 导航系统
bash
运行
ros2 launch originbot_nav2_competition nav2_competition.launch.py \
  map:=/root/ros2_ws/maps/originbot_competition_map.yaml
### 4. 初始位姿发布与任务执行
bash
运行
# 发布初始位姿 (x=-0.373, y=-0.222, yaw=31.2°)
ros2 run originbot_nav2_competition publish_initial_pose

# 启动比赛任务节点
ros2 run originbot_nav2_competition nav2_mission_node

# 团队成员
颜智琳、邹希桐、王鑫
