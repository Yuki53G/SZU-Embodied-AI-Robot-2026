# OriginBot Nav2 Competition Package

这个包用于单独测试 Nav2：导航和避障交给 Nav2，任务节点只发送目标点和识别二维码。

关键原则：
- Nav2 不知道地面颜色。
- 如果黄色区域绝对不能踩绿色，必须创建一张 Nav2 专用地图：把绿色禁区涂黑/occupied，只保留黄色通道为 free。
- 不要同时运行旧的 slam_waypoint_mission_node.py，否则两个节点会抢 /cmd_vel。

启动顺序：
1. 底盘雷达相机
2. Nav2 bringup
3. publish_initial_pose
4. 确认 map->base_link
5. nav2_mission_node
