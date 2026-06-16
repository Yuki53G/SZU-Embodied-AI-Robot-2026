#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
nav2_mission_node_nav2_primary_continuous_qr.py

OriginBot 智慧医疗赛项：Nav2 为主 + 全程扫码 + USB 扬声器播报

核心策略：
1. 不再使用 OpenCV 绕圈，C 区绕行完全交给 Nav2 waypoint；出口段加入折线引导点，避免斜线穿绿色。
2. 从出发点去 QR_POINT 的路上持续扫码：
   - 一旦扫到二维码，立即取消 QR_POINT goal，直接去 B_ENTRY / C_ENTRY / C 区路线。
   - 扫到后播报识别出的方向：clockwise.wav / anticlockwise.wav。
   - 到 QR_POINT 还没扫到时，先原地大幅摆头搜索几秒；仍扫不到再播报手动默认方向继续。
3. Nav2 为主，但不能让关键返回点太早超时：C_EXIT_CHANNEL / P_RETURN_NAV 给更长时间。
   - 普通 C 区点失败/超时：最多重试一次，再跳过该点。
   - C_EXIT_CHANNEL / P_RETURN_NAV：优先重试和回家兜底，尽量保证小车回到出发区。
6. 比赛当天可以提前用手机扫码，然后手动修改 default_direction 为 clockwise / anticlockwise。
4. 任意时刻只让 Nav2 发布 /cmd_vel，本节点不再进行黄绿视觉控车。
5. USB 扬声器固定使用 plughw:2,0，可播放 sounds/clockwise.wav 与 sounds/anticlockwise.wav。
"""

import math
import os
import time
import json
import urllib.request
import urllib.error
import shutil
import subprocess
import sys
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose


# ============================================================
# 现场只改这里
# ============================================================
USER_DEFAULT_DIRECTION = 'anticlockwise'      # clockwise=顺时针；anticlockwise=逆时针
USER_OFFICIAL_QR_RESULT_TOPIC = '/qr_code_result'  # 保留官方/外部二维码结果话题
USER_USE_OPENCV_FALLBACK = False          # 上位机方案：本机不做重解码，只把照片发给电脑

# ============================================================
# 上位机二维码方案：QR_POINT 到达后，小车原地扫码转 20 秒
# 小车把 /image 压缩图像 POST 到电脑 Python 服务，电脑解码后回传文本
# ============================================================
USER_PC_QR_ENABLE = True

# 改成你电脑的局域网 IP。Windows 用 ipconfig 看 WLAN/以太网 IPv4。
# 例：电脑 IP 是 192.168.108.123，则这里填 http://192.168.108.123:8899/qr
USER_PC_QR_URL = 'http://192.168.108.61:8899/qr'

USER_PC_QR_UPLOAD_INTERVAL_SEC = 0.30     # 每 0.30 秒发一张照片到电脑
USER_PC_QR_HTTP_TIMEOUT_SEC = 0.55        # 单次 HTTP 等待别太久，避免阻塞后台线程
USER_PC_QR_DECIDE_AFTER_FULL_SEARCH = True  # True=转满20秒后再采用电脑结果
USER_QR_SEARCH_TIMEOUT_SEC = 20.0         # 扫码点原地转动 20 秒


def normalize_direction(direction):
    d = str(direction).strip().lower()
    if d in ['anticlockwise', 'anti', 'counterclockwise', 'counter', '逆', '逆时针']:
        return 'anticlockwise'
    return 'clockwise'


def direction_cn(direction):
    return '逆时针' if normalize_direction(direction) == 'anticlockwise' else '顺时针'


def classify_qr_text(raw):
    raw = str(raw).strip()
    if not raw:
        return '空内容'
    digits = [c for c in raw if c.isdigit()]
    letters = [c for c in raw if c.isalpha()]
    lower = raw.lower()
    if len(digits) == 4 and len(digits) == len(raw):
        return '四位数字码'
    if digits and not letters:
        return '数字码'
    if 'clock' in lower or 'anti' in lower or 'counter' in lower or '顺' in raw or '逆' in raw:
        return '方向单词码'
    if digits and letters:
        return '数字+字母混合码'
    if letters:
        return '单词/文本码'
    return '其他文本码'


def yaw_to_quaternion(yaw_deg):
    yaw = math.radians(float(yaw_deg))
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


class Nav2MissionNode(Node):
    def __init__(self):
        super().__init__('nav2_mission_node')

        # ============================================================
        # 1. waypoint：当前地图下的坐标
        # ============================================================
        self.waypoints = {
            'P_START':      {'x': -0.373, 'y': -0.222, 'yaw_deg': 31.2},
            'QR_POINT':     {'x': 3.201-0.300,  'y': 1.625-0.300,  'yaw_deg': 70.3},
            'B_ENTRY':      {'x': 1.065+0.020,  'y': 1.628+0.200,  'yaw_deg': 87.6+0.4},
            'C_ENTRY':      {'x': 1.100,  'y': 2.569,  'yaw_deg': 86.7},

            # C 区四角：Nav2 负责按顺/逆时针跑点，不再用 OpenCV 绕圈。
            'C_LEFT_DOWN':  {'x': -0.836, 'y': 2.535+0.400,  'yaw_deg': 118.0},
            'C_LEFT_MID':   {'x': -1.050, 'y': 3.050,  'yaw_deg': 100.0},
            'C_LEFT_UP':    {'x': -1.241, 'y': 3.637-0.400,  'yaw_deg': 15.4},

            # 顶部加密点：不涂黑地图时，用更密的外圈点减少 Nav2 抄近路穿绿色。
            'C_TOP_LEFT':   {'x': -0.650, 'y': 4.180,  'yaw_deg': 25.0},
            'C_TOP_MID':    {'x': 0.450,  'y': 4.620-0.300,  'yaw_deg': 0.0},
            'C_TOP_RIGHT':  {'x': 1.520,  'y': 4.610-0.200,  'yaw_deg': -25.0},

            # 右上点按现场反馈：往右下微调，减少歪点和压绿。
            'C_RIGHT_UP':   {'x': 2.300,  'y': 4.580-0.300,  'yaw_deg': -55.0},
            'C_RIGHT_MID':  {'x': 2.450,  'y': 4.150-0.300,  'yaw_deg': -70.0},
            'C_RIGHT_DOWN': {'x': 2.530,  'y': 3.701-0.000,  'yaw_deg': -60.0},

            # 出通道引导点：不要从 C_RIGHT_DOWN / C_LEFT_DOWN 斜线直插 C_EXIT_CHANNEL，
            # 否则 Nav2 可能抄近路压绿色。这里用小段折线把车引到通道口。
            'C_EXIT_RIGHT_GUIDE': {'x': 2.050, 'y': 2.850+0.200, 'yaw_deg': -125.0},
            'C_EXIT_MID_GUIDE':   {'x': 1.430, 'y': 2.420+0.300, 'yaw_deg': -125.0},
            'C_EXIT_PRE_CHANNEL': {'x': 1.060, 'y': 2.180+0.500, 'yaw_deg': -92.0},
            'C_EXIT_LEFT_GUIDE':  {'x': -0.350, 'y': 2.460+0.300, 'yaw_deg': -20.0},
            'C_EXIT_LEFT_PRE':    {'x': 0.420+0.200,  'y': 2.220+0.400, 'yaw_deg': -35.0},

            # 绕完后先去出通道口。
            'C_EXIT_CHANNEL': {'x': 0.950, 'y': 1.773, 'yaw_deg': -90.7},

            # 出通道后新增一个回家引导点：
            # 目的：不要从 C_EXIT_CHANNEL 直接斜插 P_RETURN_NAV，先沿通道往下走一段，避开违禁区域边缘。
            'P_RETURN_GATE': {'x': 0.760, 'y': 0.750, 'yaw_deg': -112.0},

            # 返回粗目标：你们现场新调的点。
            'P_RETURN_NAV': {'x': -0.630, 'y': -0.622, 'yaw_deg': -98.4},
        }

        # ============================================================
        # 2. 状态机与 Nav2 action 管理
        # ============================================================
        self.state = 'WAIT_NAV2'
        self.nav_queue = []
        self.after_nav_state = None
        self.current_goal_name = None
        self.goal_send_time = 0.0
        self.goal_active = False
        self.goal_handle = None
        self.result_future = None

        # token 防止 cancel 后旧回调影响新任务。
        self.goal_token = 0
        self.active_goal_token = None

        # 规划/到点等待不要太久：超时就跳下一个点。
        self.goal_timeout = {
            'QR_POINT': 40.0,
            'B_ENTRY': 20.0,
            'C_ENTRY': 22.0,
            'C_LEFT_DOWN': 22.0,
            'C_LEFT_MID': 18.0,
            'C_LEFT_UP': 24.0,
            'C_TOP_LEFT': 18.0,
            'C_TOP_MID': 18.0,
            'C_TOP_RIGHT': 18.0,
            'C_RIGHT_UP': 26.0,
            'C_RIGHT_MID': 18.0,
            'C_RIGHT_DOWN': 24.0,
            'C_EXIT_RIGHT_GUIDE': 14.0,
            'C_EXIT_MID_GUIDE': 14.0,
            'C_EXIT_PRE_CHANNEL': 12.0,
            'C_EXIT_LEFT_GUIDE': 14.0,
            'C_EXIT_LEFT_PRE': 14.0,
            'C_EXIT_CHANNEL': 26.0,
            'P_RETURN_GATE': 24.0,
            'P_RETURN_NAV': 42.0,
            'P_START': 35.0,
            'default': 20.0,
        }

        # 失败计数：关键点不再一失败就结束，尤其返回点必须尽量回。
        self.fail_counts = {}

        # ============================================================
        # 3. 二维码：全程扫，扫到直接走入口；扫不到则播报默认方向
        # ============================================================
        self.qr_detector = cv2.QRCodeDetector()
        # 比赛当天可提前用手机扫二维码，然后手动改这里：
        #   'clockwise'     = 顺时针
        #   'anticlockwise' = 逆时针
        # 当前现场默认：扫码失败默认顺时针。
        self.default_direction = normalize_direction(USER_DEFAULT_DIRECTION)
        self.mission_direction = None
        self.qr_locked = False
        self.last_qr_text = ''
        self.last_qr_time = 0.0

        # 官方二维码包 originbot_qrcode_detect 融合：
        # qr_decoder.cpp 里发布 std_msgs/String 到 qr_code_result。
        # 本节点优先订阅官方结果；如果官方没有输出，再保留下面 OpenCV fallback。
        self.official_qr_result_topic = USER_OFFICIAL_QR_RESULT_TOPIC
        self.use_opencv_fallback = bool(USER_USE_OPENCV_FALLBACK)
        self.last_official_qr_text = ''
        self.last_official_qr_time = 0.0
        self.last_qr_terminal_preview_time = 0.0

        # 上位机扫码：image_callback 只存最新 JPEG，后台线程负责发给电脑，不阻塞主状态机。
        self.pc_qr_enable = bool(USER_PC_QR_ENABLE)
        self.pc_qr_url = str(USER_PC_QR_URL).strip()
        self.pc_qr_upload_interval = float(USER_PC_QR_UPLOAD_INTERVAL_SEC)
        self.pc_qr_http_timeout = float(USER_PC_QR_HTTP_TIMEOUT_SEC)
        self.pc_qr_decide_after_full_search = bool(USER_PC_QR_DECIDE_AFTER_FULL_SEARCH)

        self.latest_jpeg_lock = threading.Lock()
        self.latest_jpeg_bytes = None
        self.latest_jpeg_time = 0.0

        self.pc_result_lock = threading.Lock()
        self.pending_pc_qr_text = ''
        self.pending_pc_qr_time = 0.0
        self.pending_pc_qr_source = ''

        self.pc_qr_stop_event = threading.Event()
        self.pc_qr_thread = threading.Thread(target=self.pc_qr_upload_loop, daemon=True)
        self.pc_qr_thread.start()

        # QR 调试与增强解码：
        # 如果现场一直扫不到，代码会每隔一段时间把当前画面保存到 /tmp/originbot_qr_debug/last_qr_frame.jpg
        # 便于判断到底是相机没看到二维码，还是看到了但 OpenCV 没解码出来。
        self.qr_debug_enabled = True
        self.qr_debug_dir = '/tmp/originbot_qr_debug'
        self.qr_last_debug_save_time = 0.0
        self.qr_debug_save_interval = 1.0
        self.qr_frame_count = 0

        # QR_POINT 到达后，如果还没扫到码，原地大幅摆头搜索几秒，尽量扫上。
        # 这一步只在 QR_POINT 到达但未识别二维码时启用；路上仍然一直由 image_callback 持续扫码。
        self.qr_search_start_time = 0.0

        # 扫码搜索时间加长：原来 8s 太短，现场看得到但解不出来时，给更多稳定视角。
        self.qr_search_timeout = float(USER_QR_SEARCH_TIMEOUT_SEC)

        # 扫码点原地搜索参数：
        # 目标：慢一点、转角更大、向左扫得更多。
        # 说明：ROS 中一般 angular.z > 0 是向左转；如果你们实车方向相反，把 left/right 的正负号对调。
        self.qr_search_vx = 0.0

        # 先轻微后退，让二维码不要贴得太近，避免过大/失焦。
        self.qr_search_backup_time = 2.00
        self.qr_search_backup_vx = -0.040
        self.qr_search_backup_wz = 0.12

        # 大角度慢速摆头：左扫时间更长、角速度略大；右扫只回一点。
        self.qr_search_left_wz = 0.32
        self.qr_search_right_wz = -0.22
        self.qr_search_left_time = 4.20
        self.qr_search_right_time = 2.60
        self.qr_search_hold_time = 0.45

        # 手动起跑门：节点先启动、订阅相机、等待 Nav2 ready；终端按空格/回车后才真正发第一个 goal。
        self.wait_for_manual_start = True
        self.manual_start_received = False
        self.nav2_ready = False
        self._start_input_thread()

        # ============================================================
        # 4. 播报：优先播放 sounds/key.wav，USB 声卡固定 plughw:2,0
        # ============================================================
        self.speech_enabled = True
        self.last_speech_time = 0.0
        self.sound_dir = '/root/ros2_ws/src/originbot_nav2_competition/sounds'
        self.audio_device = 'plughw:0,0'

        # ============================================================
        # 5. ROS 通信
        # ============================================================
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.image_sub = self.create_subscription(
            CompressedImage, '/image', self.image_callback, sensor_qos
        )

        # 官方二维码识别包输出：originbot_qrcode_detect/qr_decoder.cpp -> std_msgs/String qr_code_result
        # 只要你另开终端启动官方 qr_decoder/qrcode_control，这里就能直接收到扫码文字。
        self.official_qr_sub = self.create_subscription(
            String, self.official_qr_result_topic, self.official_qr_result_callback, 10
        )

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().warn(
            'nav2_mission_node NAV2-PRIMARY + OFFICIAL-QR-FUSION started. '
            'Official qr_code_result preferred; OpenCV fallback kept; C loop uses dense Nav2 waypoints.'
        )
        self.log_runtime_config()
        self.log_all_waypoints()

    # ============================================================
    # 基础工具
    # ============================================================
    def log_runtime_config(self):
        self.get_logger().warn('========== RUNTIME CONFIG START ==========')
        self.get_logger().warn(
            f'DEFAULT_DIRECTION={self.default_direction}({direction_cn(self.default_direction)})'
        )
        self.get_logger().warn(
            f'OFFICIAL_QR_TOPIC={self.official_qr_result_topic}  '
            f'USE_OPENCV_FALLBACK={self.use_opencv_fallback}'
        )
        self.get_logger().warn(
            'QR digit rule: last digit odd -> clockwise(顺时针), '
            'even -> anticlockwise(逆时针)'
        )
        self.get_logger().warn(
            'QR word rule: A/anti/counter/逆 -> anticlockwise; '
            'C/clockwise/顺 -> clockwise'
        )
        self.get_logger().warn(
            f'QR_SEARCH: timeout={self.qr_search_timeout:.1f}s, '
            f'backup={self.qr_search_backup_time:.1f}s vx={self.qr_search_backup_vx:.3f}, '
            f'left_wz={self.qr_search_left_wz:.2f} left_time={self.qr_search_left_time:.1f}s, '
            f'right_wz={self.qr_search_right_wz:.2f} right_time={self.qr_search_right_time:.1f}s'
        )
        self.get_logger().warn(
            f'PC_QR_ENABLE={self.pc_qr_enable} url={self.pc_qr_url} '
            f'upload_interval={self.pc_qr_upload_interval:.2f}s '
            f'http_timeout={self.pc_qr_http_timeout:.2f}s '
            f'decide_after_full_search={self.pc_qr_decide_after_full_search}'
        )
        self.get_logger().warn('========== RUNTIME CONFIG END ==========')

    def print_qr_terminal_result(self, decoded, source, raw_text, direction, method=''):
        direction = normalize_direction(direction)
        raw_text = str(raw_text).strip() if raw_text is not None else ''
        qr_type = classify_qr_text(raw_text) if decoded else '未解码'
        decoded_str = 'YES' if decoded else 'NO'
        self.get_logger().warn('========== QR TERMINAL RESULT ==========')
        self.get_logger().warn(f'decoded={decoded_str} source={source} method={method}')
        self.get_logger().warn(f'qr_text=[{raw_text if raw_text else "NO_QR_DECODED"}] type={qr_type}')
        self.get_logger().warn(
            f'direction={direction}({direction_cn(direction)}) '
            f'default={self.default_direction}({direction_cn(self.default_direction)})'
        )
        self.get_logger().warn('========================================')

    def print_qr_default_preview(self, source):
        now = time.time()
        if now - self.last_qr_terminal_preview_time < 1.5:
            return
        self.last_qr_terminal_preview_time = now
        self.get_logger().warn(
            f'[QR_PREVIEW] decoded=NO source={source} -> '
            f'use default={self.default_direction}({direction_cn(self.default_direction)})'
        )

    def log_all_waypoints(self):
        """启动时打印所有 waypoint 坐标，现场方便核对点位。"""
        try:
            self.get_logger().warn('========== WAYPOINT TABLE START ==========')
            for name, p in self.waypoints.items():
                self.get_logger().warn(
                    f"{name}: x={float(p['x']):.3f}, y={float(p['y']):.3f}, yaw={float(p['yaw_deg']):.1f}"
                )
            self.get_logger().warn('========== WAYPOINT TABLE END ==========')
        except Exception as e:
            self.get_logger().warn(f'log_all_waypoints error: {e}')

    def stop_cmd(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd

    def publish_stop(self):
        self.cmd_pub.publish(self.stop_cmd())

    def _start_input_thread(self):
        """后台等待终端输入。

        目的：先把 ROS 节点、相机订阅、Nav2 action client 都暖起来，
        比赛发车时在终端按空格/回车即可真正开始任务。
        """
        def wait_for_enter():
            try:
                print('\n[READY GATE] Mission node is loaded. Press SPACE then ENTER, or just ENTER, to start.\n', flush=True)
                sys.stdin.readline()
                self.manual_start_received = True
                try:
                    self.get_logger().warn('Manual start received from terminal.')
                except Exception:
                    pass
            except Exception as e:
                try:
                    self.get_logger().warn(f'Manual start input thread error: {e}. Auto-start disabled until flag set.')
                except Exception:
                    pass

        t = threading.Thread(target=wait_for_enter, daemon=True)
        t.start()

    def start_first_goal_after_gate(self):
        """手动确认后真正开始。

        如果等待期间已经扫到了二维码，则直接进 C 区路线；
        如果没扫到，则按原逻辑先去 QR_POINT，路上继续扫码。
        """
        if self.qr_locked:
            self.get_logger().warn('Manual start: QR already locked before moving. Start mission route now.')
            self.start_mission_route_after_qr()
            return

        self.get_logger().warn('Manual start: Send QR_POINT, scan QR continuously on the way.')
        self.state = 'NAV_TO_QR'
        self.send_goal('QR_POINT')

    def play_or_speak(self, key, chinese_text, english_text):
        """优先播放自定义 wav；没有 wav 再用 espeak-ng 生成临时 wav 后走 USB 声卡。"""
        if not self.speech_enabled:
            return
        now = time.time()
        if now - self.last_speech_time < 1.0:
            return
        self.last_speech_time = now

        try:
            # 1. 自定义 wav：/sounds/clockwise.wav / anticlockwise.wav
            wav_path = os.path.join(self.sound_dir, f'{key}.wav')
            if os.path.exists(wav_path) and shutil.which('aplay') is not None:
                cmd = ['aplay', '-q', '-D', self.audio_device, wav_path]
                subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self.get_logger().warn(f'Play wav: {wav_path} device={self.audio_device}')
                return

            # 2. 没有 wav 时，用 espeak-ng 生成临时 wav，再 aplay 到 USB。
            if shutil.which('espeak-ng') is not None and shutil.which('aplay') is not None:
                tmp_wav = f'/tmp/originbot_tts_{key}.wav'
                subprocess.run(
                    ['espeak-ng', '-v', 'zh', '-w', tmp_wav, chinese_text],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=2.0,
                )
                if os.path.exists(tmp_wav):
                    subprocess.Popen(
                        ['aplay', '-q', '-D', self.audio_device, tmp_wav],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    self.get_logger().warn(f'Speak zh via USB: {chinese_text}')
                    return

            # 3. 最后兜底：默认设备。
            if shutil.which('espeak-ng') is not None:
                subprocess.Popen(
                    ['espeak-ng', '-v', 'zh', chinese_text],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                self.get_logger().warn(f'Speak zh default: {chinese_text}')
                return

            if shutil.which('espeak') is not None:
                subprocess.Popen(
                    ['espeak', english_text],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                self.get_logger().warn(f'Speak en default: {english_text}')
                return

            self.get_logger().warn('No aplay/espeak-ng/espeak found. Speech skipped.')
        except Exception as e:
            self.get_logger().warn(f'play_or_speak error: {e}')

    def announce_direction(self, direction, reason=''):
        """只播报最终执行方向，不播报扫码失败。

        扫码成功：播报扫到的方向。
        扫码失败/超时：播报 default_direction 对应方向。
        """
        if direction == 'anticlockwise':
            self.get_logger().warn(f'Announce direction: anticlockwise reason={reason}')
            self.play_or_speak('anticlockwise', '逆时针', 'anti clockwise')
        else:
            self.get_logger().warn(f'Announce direction: clockwise reason={reason}')
            self.play_or_speak('clockwise', '顺时针', 'clockwise')

    # ============================================================
    # 上位机二维码：存图、发图、接收电脑解码结果
    # ============================================================
    def update_latest_jpeg_from_image_msg(self, msg):
        """保存最新一帧压缩图像。

        当前 mission_node 订阅的是 sensor_msgs/msg/CompressedImage 的 /image。
        msg.data 通常就是 JPEG 字节，可以直接发给电脑。
        这里不做 cv2 解码，避免拖慢小车主状态机。
        """
        try:
            if msg is None or not hasattr(msg, 'data') or len(msg.data) <= 0:
                return
            with self.latest_jpeg_lock:
                self.latest_jpeg_bytes = bytes(msg.data)
                self.latest_jpeg_time = time.time()
        except Exception as e:
            self.get_logger().warn(f'update_latest_jpeg_from_image_msg error: {e}', throttle_duration_sec=1.0)

    def set_pending_pc_qr_result(self, text, source='pc_http'):
        text = str(text).strip()
        if not text:
            return
        with self.pc_result_lock:
            # 已经有结果就保留第一个，防止后续误识别覆盖。
            if not self.pending_pc_qr_text:
                self.pending_pc_qr_text = text
                self.pending_pc_qr_time = time.time()
                self.pending_pc_qr_source = source
                self.get_logger().warn(f'PC_QR pending result saved: text=[{text}] source={source}')

    def take_pending_pc_qr_result(self):
        with self.pc_result_lock:
            text = self.pending_pc_qr_text
            ts = self.pending_pc_qr_time
            source = self.pending_pc_qr_source
            self.pending_pc_qr_text = ''
            self.pending_pc_qr_time = 0.0
            self.pending_pc_qr_source = ''
        return text, ts, source

    def peek_pending_pc_qr_result(self):
        with self.pc_result_lock:
            return self.pending_pc_qr_text, self.pending_pc_qr_time, self.pending_pc_qr_source

    def pc_qr_upload_loop(self):
        """后台线程：QR_SEARCH 期间持续把最新照片发给电脑。

        注意：这个线程不直接改 Nav2 状态，不直接 cancel goal。
        它只把电脑返回的二维码文本存成 pending_pc_qr_text，
        真正采用结果由 control_loop / handle_qr_search 执行，避免主循环被 HTTP 阻塞。
        """
        last_upload = 0.0

        while not getattr(self, 'pc_qr_stop_event', threading.Event()).is_set():
            try:
                time.sleep(0.03)

                if not getattr(self, 'pc_qr_enable', False):
                    continue
                if self.qr_locked:
                    continue
                # 只在 QR_SEARCH 阶段上传照片，满足“到扫码点转20秒再扫”的比赛策略。
                if self.state != 'QR_SEARCH':
                    continue

                now = time.time()
                if now - last_upload < self.pc_qr_upload_interval:
                    continue
                last_upload = now

                with self.latest_jpeg_lock:
                    img_bytes = self.latest_jpeg_bytes
                    img_time = self.latest_jpeg_time

                if not img_bytes:
                    self.get_logger().warn('PC_QR no latest JPEG frame yet.', throttle_duration_sec=1.0)
                    continue

                # 避免上传很旧的帧。
                if now - img_time > 2.0:
                    self.get_logger().warn(
                        f'PC_QR latest JPEG is old age={now - img_time:.1f}s',
                        throttle_duration_sec=1.0
                    )
                    continue

                req = urllib.request.Request(
                    self.pc_qr_url,
                    data=img_bytes,
                    headers={
                        'Content-Type': 'image/jpeg',
                        'X-OriginBot-Time': f'{now:.3f}',
                    },
                    method='POST',
                )

                try:
                    with urllib.request.urlopen(req, timeout=self.pc_qr_http_timeout) as resp:
                        body = resp.read(4096).decode('utf-8', errors='ignore')
                except Exception as e:
                    self.get_logger().warn(f'PC_QR HTTP send failed: {e}', throttle_duration_sec=1.0)
                    continue

                try:
                    payload = json.loads(body)
                except Exception:
                    self.get_logger().warn(f'PC_QR bad JSON response: {body[:120]}', throttle_duration_sec=1.0)
                    continue

                decoded = bool(payload.get('decoded', False))
                text = str(payload.get('text', '')).strip()
                method = str(payload.get('method', 'pc')).strip()

                if decoded and text:
                    self.get_logger().warn(f'PC_QR HTTP decoded text=[{text}] method={method}')
                    self.set_pending_pc_qr_result(text, source=f'pc_http_{method}')

            except Exception as e:
                try:
                    self.get_logger().warn(f'pc_qr_upload_loop error: {e}', throttle_duration_sec=1.0)
                except Exception:
                    pass

    def use_pc_qr_result_now(self, reason=''):
        """把电脑已经识别出的 pending 结果正式用于路线选择。"""
        qr_text, ts, source = self.take_pending_pc_qr_result()
        if not qr_text:
            return False

        direction = self.decide_direction_from_qr(qr_text)
        if direction is None:
            direction = self.default_direction

        self.last_qr_text = qr_text
        self.last_qr_time = ts if ts > 0 else time.time()

        self.print_qr_terminal_result(
            decoded=True,
            source='PC_HTTP_QR_READER',
            raw_text=qr_text,
            direction=direction,
            method=f'{source}; reason={reason}',
        )

        self.qr_locked = True
        self.mission_direction = direction

        self.get_logger().warn(
            f'PC QR SUCCESS text=[{qr_text}] direction={direction}({direction_cn(direction)}). '
            f'reason={reason}. Start route now.'
        )

        self.announce_direction(direction, reason='pc_qr_success')
        self.publish_stop()
        self.start_mission_route_after_qr()
        return True

    # ============================================================
    # 官方二维码结果回调：优先使用 originbot_qrcode_detect 的 qr_code_result
    # ============================================================
    def official_qr_result_callback(self, msg):
        if self.qr_locked:
            return

        try:
            qr_text = str(msg.data).strip()
        except Exception:
            qr_text = ''

        if not qr_text:
            self.print_qr_default_preview('OFFICIAL_QR_EMPTY')
            return

        self.last_official_qr_text = qr_text
        self.last_official_qr_time = time.time()
        self.last_qr_text = qr_text
        self.last_qr_time = self.last_official_qr_time

        direction = self.decide_direction_from_qr(qr_text)
        if direction is None:
            direction = self.default_direction

        self.print_qr_terminal_result(
            decoded=True,
            source='OFFICIAL_originbot_qrcode_detect',
            raw_text=qr_text,
            direction=direction,
            method=self.official_qr_result_topic,
        )

        self.lock_qr_direction(direction, qr_text)

    # ============================================================
    # 图像回调：持续扫码
    # ============================================================
    def image_callback(self, msg):
        if self.qr_locked:
            return

        # 上位机扫码方案：先保存最新 JPEG，后台线程会在 QR_SEARCH 阶段发给电脑。
        self.update_latest_jpeg_from_image_msg(msg)

        if not self.use_opencv_fallback:
            self.print_qr_default_preview('WAIT_PC_HTTP_QR_OR_OFFICIAL_QR_RESULT')
            return

        try:
            self.qr_frame_count += 1

            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn('QR image_callback: cv2.imdecode returned None.', throttle_duration_sec=2.0)
                return

            qr_text = self.detect_qr_multi(frame)
            if not qr_text:
                # OpenCV fallback 没扫到：终端显示默认方向，同时保存一张最新画面。
                self.print_qr_default_preview('OPENCV_FALLBACK_NO_DECODE')
                self.save_qr_debug_frame(frame)
                return

            self.last_qr_text = qr_text.strip()
            self.last_qr_time = time.time()
            self.get_logger().warn(f'QR raw decoded text=[{self.last_qr_text}]')

            direction = self.decide_direction_from_qr(self.last_qr_text)
            if direction is None:
                # 扫到了但解析不出方向时，默认顺时针，但仍然认为扫码成功。
                direction = self.default_direction
                self.get_logger().warn(
                    f'QR decoded but direction unknown. Use default_direction={self.default_direction}.'
                )

            self.print_qr_terminal_result(
                decoded=True,
                source='OPENCV_FALLBACK',
                raw_text=self.last_qr_text,
                direction=direction,
                method='detect_qr_multi',
            )

            self.lock_qr_direction(direction, self.last_qr_text)

        except Exception as e:
            self.get_logger().warn(f'image_callback error: {e}', throttle_duration_sec=1.0)

    def save_qr_debug_frame(self, frame):
        """保存最近一次没有解码成功的相机画面，方便现场排查二维码是否真的在画面里。"""
        if not self.qr_debug_enabled:
            return

        now = time.time()
        if now - self.qr_last_debug_save_time < self.qr_debug_save_interval:
            return
        self.qr_last_debug_save_time = now

        try:
            os.makedirs(self.qr_debug_dir, exist_ok=True)

            raw_path = os.path.join(self.qr_debug_dir, 'last_qr_frame.jpg')
            cv2.imwrite(raw_path, frame)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray_path = os.path.join(self.qr_debug_dir, 'last_qr_gray.jpg')
            cv2.imwrite(gray_path, gray)

            h, w = gray.shape[:2]
            x1, x2 = int(w * 0.18), int(w * 0.82)
            y1, y2 = int(h * 0.18), int(h * 0.82)
            crop = gray[y1:y2, x1:x2]
            if crop.size > 0:
                crop_path = os.path.join(self.qr_debug_dir, 'last_qr_center_crop.jpg')
                cv2.imwrite(crop_path, crop)

            self.get_logger().warn(
                f'QR not decoded yet. Saved debug frame: {raw_path} '
                f'shape={frame.shape} frame_count={self.qr_frame_count}',
                throttle_duration_sec=2.0,
            )
        except Exception as e:
            self.get_logger().warn(f'save_qr_debug_frame error: {e}', throttle_duration_sec=2.0)

    def decode_qr_once(self, img, tag=''):
        """对单张图尝试 OpenCV QR 解码，兼容单码和多码接口。"""
        try:
            data, _, _ = self.qr_detector.detectAndDecode(img)
            if data:
                self.get_logger().warn(f'QR decoded by detectAndDecode tag={tag} data=[{data}]')
                return data
        except Exception:
            pass

        try:
            if hasattr(self.qr_detector, 'detectAndDecodeMulti'):
                ok, decoded_info, _, _ = self.qr_detector.detectAndDecodeMulti(img)
                if ok and decoded_info:
                    for data in decoded_info:
                        if data:
                            self.get_logger().warn(f'QR decoded by detectAndDecodeMulti tag={tag} data=[{data}]')
                            return data
        except Exception:
            pass

        return ''

    def detect_qr_multi(self, frame):
        """增强版二维码识别。

        目标不是改变任务逻辑，而是提高“已经看见二维码时”的解码概率：
        1. 原图直接解码；
        2. 灰度/直方图均衡/CLAHE 增强；
        3. 中心裁剪，避免背景干扰；
        4. 放大图像，帮助小二维码；
        5. 自适应阈值/反色，处理光照问题。
        """
        try:
            if frame is None:
                return ''

            h, w = frame.shape[:2]
            candidates = []

            # 原图
            candidates.append(('bgr_full', frame))

            # 中心裁剪：二维码通常在正前方，裁剪能减少背景干扰。
            for ratio in [0.82, 0.65]:
                x1 = int(w * (1.0 - ratio) / 2.0)
                x2 = int(w * (1.0 + ratio) / 2.0)
                y1 = int(h * (1.0 - ratio) / 2.0)
                y2 = int(h * (1.0 + ratio) / 2.0)
                crop = frame[y1:y2, x1:x2]
                if crop.size > 0:
                    candidates.append((f'bgr_center_{int(ratio * 100)}', crop))

            # 先对 BGR 原图/裁剪图直接解码和放大解码。
            for tag, img in candidates:
                data = self.decode_qr_once(img, tag)
                if data:
                    return data

                for scale in [1.6, 2.2]:
                    big = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
                    data = self.decode_qr_once(big, f'{tag}_x{scale}')
                    if data:
                        return data

            # 灰度增强候选。
            gray_candidates = []
            for tag, img in candidates:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                gray_candidates.append((f'{tag}_gray', gray))

                eq = cv2.equalizeHist(gray)
                gray_candidates.append((f'{tag}_eq', eq))

                clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
                gray_candidates.append((f'{tag}_clahe', clahe))

                # 轻微锐化，处理边缘发虚。
                kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]], dtype=np.float32)
                sharp = cv2.filter2D(gray, -1, kernel)
                gray_candidates.append((f'{tag}_sharp', sharp))

                # Otsu 二值化、反色、自适应阈值，处理亮度不均。
                _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                gray_candidates.append((f'{tag}_otsu', otsu))
                gray_candidates.append((f'{tag}_otsu_inv', cv2.bitwise_not(otsu)))

                adapt = cv2.adaptiveThreshold(
                    gray, 255,
                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    cv2.THRESH_BINARY,
                    31, 5
                )
                gray_candidates.append((f'{tag}_adaptive', adapt))
                gray_candidates.append((f'{tag}_adaptive_inv', cv2.bitwise_not(adapt)))

            for tag, img in gray_candidates:
                data = self.decode_qr_once(img, tag)
                if data:
                    return data

                for scale in [1.8, 2.5]:
                    big = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
                    data = self.decode_qr_once(big, f'{tag}_x{scale}')
                    if data:
                        return data

        except Exception as e:
            self.get_logger().warn(f'detect_qr_multi error: {e}', throttle_duration_sec=1.0)
            return ''

        return ''

    def decide_direction_from_qr(self, text):
        if not text:
            return None

        raw = str(text).strip()
        lower = raw.lower()

        # 新增判断：如果二维码内容类似 ClockWise / clockwise / Cxxx，
        # 或者 anticlockwise / AntiClockWise / Axxx，就优先看首位。
        # 这样不会替代原来的 anti/counter/中文/数字奇偶逻辑，只是在前面多一层更直接的判断。
        first_alpha = ''
        for ch in lower:
            if ch.isalpha():
                first_alpha = ch
                break

        if first_alpha == 'a':
            return 'anticlockwise'
        if first_alpha == 'c':
            return 'clockwise'

        # 原有判断保留：支持英文关键词、中文关键词。
        if 'anti' in lower or 'counter' in lower or '逆' in raw:
            return 'anticlockwise'
        if 'clockwise' in lower or '顺' in raw:
            return 'clockwise'

        # 数字码判断：
        # 现场规则：扫到 4 位数字/数字码，看最后一位：
        #   奇数 -> 顺时针 clockwise
        #   偶数 -> 逆时针 anticlockwise
        # 这里不强制必须刚好 4 位，避免二维码里混入空格/换行时失效；
        # 只要识别文本里有数字，就取最后一个数字作为方向码。
        digits = [c for c in raw if c.isdigit()]
        if digits:
            last_digit = int(digits[-1])
            direction = 'clockwise' if last_digit % 2 == 1 else 'anticlockwise'
            self.get_logger().warn(
                f'QR numeric code detected raw=[{raw}] last_digit={last_digit} '
                f'rule=odd_clockwise_even_anticlockwise direction={direction}'
            )
            return direction
        return None

    def lock_qr_direction(self, direction, qr_text):
        if self.qr_locked:
            return
        self.qr_locked = True
        self.mission_direction = direction

        self.get_logger().warn(f'QR SUCCESS text=[{qr_text}] direction={direction}. Go to entrance now.')

        # 扫码成功：播报扫到的最终执行方向。
        self.announce_direction(direction, reason='qr_success')

        # 如果正在去 QR_POINT，或者已经进入 QR_SEARCH 搜码阶段，
        # 扫到后都立即取消当前动作/停车，并直接去入口和 C 区路线。
        if self.state in ['NAV_TO_QR', 'QR_SEARCH']:
            self.cancel_active_goal('qr detected, start route now')
            self.publish_stop()
            self.start_mission_route_after_qr()

    # ============================================================
    # Nav2 action 管理
    # ============================================================
    def goal_yaw_deg(self, name):
        """Nav2 连续跑点时，目标朝向尽量朝向下一个 waypoint，减少到点后原地转向。

        这样顺/逆时针使用同一组 waypoint 时，朝向会随路线方向自动变化，
        比固定 yaw_deg 更适合比赛赶时间。
        """
        if self.state == 'NAV_QUEUE' and self.nav_queue and name in self.nav_queue:
            idx = self.nav_queue.index(name)
            if idx + 1 < len(self.nav_queue):
                p = self.waypoints[name]
                q = self.waypoints[self.nav_queue[idx + 1]]
                dx = float(q['x']) - float(p['x'])
                dy = float(q['y']) - float(p['y'])
                if abs(dx) + abs(dy) > 1e-6:
                    return math.degrees(math.atan2(dy, dx))
        return float(self.waypoints[name]['yaw_deg'])

    def make_goal_pose(self, name):
        p = self.waypoints[name]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(p['x'])
        pose.pose.position.y = float(p['y'])
        pose.pose.position.z = 0.0
        yaw_deg = self.goal_yaw_deg(name)
        qz, qw = yaw_to_quaternion(yaw_deg)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.get_logger().warn(f"Goal pose {name}: x={p['x']:.3f} y={p['y']:.3f} yaw={yaw_deg:.1f}")
        return pose

    def send_goal(self, name):
        if self.goal_active:
            self.get_logger().warn(f'Goal already active: {self.current_goal_name}')
            return

        self.goal_token += 1
        token = self.goal_token
        self.active_goal_token = token

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_goal_pose(name)

        self.current_goal_name = name
        self.goal_send_time = time.time()
        self.goal_active = True

        self.get_logger().warn(f'Send Nav2 goal: {name} token={token}')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda fut, t=token, n=name: self.goal_response_callback(fut, t, n))

    def goal_response_callback(self, future, token, name):
        if token != self.active_goal_token:
            self.get_logger().warn(f'Ignore stale goal response name={name} token={token}')
            return

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Goal response error: {e}')
            self.goal_active = False
            self.active_goal_token = None
            self.on_goal_failed(name, 'response_error')
            return

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected: {name}')
            self.goal_active = False
            self.goal_handle = None
            self.active_goal_token = None
            self.on_goal_failed(name, 'rejected')
            return

        self.goal_handle = goal_handle
        self.get_logger().warn(f'Goal accepted: {name} token={token}')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(lambda fut, t=token, n=name: self.goal_result_callback(fut, t, n))

    def goal_result_callback(self, future, token, name):
        if token != self.active_goal_token:
            self.get_logger().warn(f'Ignore stale Nav2 result name={name} token={token}, state={self.state}')
            return

        try:
            result = future.result()
            status = result.status
        except Exception as e:
            self.get_logger().error(f'Goal result error: {e}')
            status = None

        self.goal_active = False
        self.goal_handle = None
        self.result_future = None
        self.current_goal_name = None
        self.active_goal_token = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Goal reached: {name}')
            self.on_goal_reached(name)
        else:
            self.get_logger().error(f'Goal failed: {name}, status={status}')
            self.on_goal_failed(name, f'status={status}')

    def cancel_active_goal(self, reason=''):
        if not self.goal_active:
            return
        self.get_logger().warn(f'Cancel active Nav2 goal: {self.current_goal_name}, reason={reason}')
        old_token = self.active_goal_token
        try:
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
        except Exception as e:
            self.get_logger().warn(f'cancel_goal_async error: {e}')

        self.goal_active = False
        self.goal_handle = None
        self.result_future = None
        self.current_goal_name = None
        self.active_goal_token = None
        self.get_logger().warn(f'Cancelled local goal token={old_token}')
        self.publish_stop()

    def start_nav_queue(self, names, after_state):
        self.nav_queue = list(names)
        self.after_nav_state = after_state
        # 新队列开始时清掉普通失败计数，避免上一次试跑影响下一次。
        self.fail_counts = {}
        if not self.nav_queue:
            self.state = after_state
            self.on_enter_state(after_state)
            return
        self.state = 'NAV_QUEUE'
        self.send_goal(self.nav_queue[0])

    def build_c_route(self):
        # 不涂黑地图时，Nav2 不知道绿色区域不能走。
        # 所以用“更密的外圈点”逼近黄色外圈路线，减少直接抄近路穿绿色。
        if self.mission_direction == 'anticlockwise':
            return [
                'B_ENTRY',
                'C_ENTRY',
                'C_RIGHT_DOWN',
                'C_RIGHT_MID',
                'C_RIGHT_UP',
                'C_TOP_RIGHT',
                'C_TOP_MID',
                'C_TOP_LEFT',
                'C_LEFT_UP',
                'C_LEFT_MID',
                'C_LEFT_DOWN',
                # 逆时针最后从左下出来：用左侧引导点进通道，避免斜线直插出口。
                'C_EXIT_LEFT_GUIDE',
                'C_EXIT_LEFT_PRE',
                'C_EXIT_PRE_CHANNEL',
                'C_EXIT_CHANNEL',
                'P_RETURN_GATE',
                'P_RETURN_NAV',
            ]
        return [
            'B_ENTRY',
            'C_ENTRY',
            'C_LEFT_DOWN',
            'C_LEFT_MID',
            'C_LEFT_UP',
            'C_TOP_LEFT',
            'C_TOP_MID',
            'C_TOP_RIGHT',
            'C_RIGHT_UP',
            'C_RIGHT_MID',
            'C_RIGHT_DOWN',
            # 顺时针最后从右下出来：用右侧/中间引导点进通道，避免斜线直插出口。
            'C_EXIT_RIGHT_GUIDE',
            'C_EXIT_MID_GUIDE',
            'C_EXIT_PRE_CHANNEL',
            'C_EXIT_CHANNEL',
            'P_RETURN_GATE',
            'P_RETURN_NAV',
        ]

    def start_mission_route_after_qr(self):
        if self.mission_direction is None:
            self.mission_direction = self.default_direction
        route = self.build_c_route()
        self.get_logger().warn(f'Start Nav2 mission route: direction={self.mission_direction}, route={route}')
        self.start_nav_queue(route, after_state='COMPLETE')

    def start_qr_search(self, reason=''):
        # 到 QR_POINT 仍未扫到二维码时，先不要马上默认，
        # 原地大幅摆头搜索几秒，尽量提高扫码成功率。
        self.cancel_active_goal('enter QR_SEARCH')
        self.publish_stop()
        self.state = 'QR_SEARCH'
        self.qr_search_start_time = time.time()
        # 新一轮 QR_SEARCH 清空上一次电脑扫码结果，防止旧码污染。
        with self.pc_result_lock:
            self.pending_pc_qr_text = ''
            self.pending_pc_qr_time = 0.0
            self.pending_pc_qr_source = ''

        self.get_logger().warn(
            f'Enter QR_SEARCH reason={reason}. '
            f'Robot will rotate/search for {self.qr_search_timeout:.1f}s and upload photos to PC. '
            f'PC url={self.pc_qr_url}. '
            'After full search, use PC decoded result if available; otherwise use default.'
        )

    def handle_qr_search(self):
        # 如果官方 qr_code_result 已经扫到并锁定方向，直接进入路线。
        if self.qr_locked:
            self.publish_stop()
            self.start_mission_route_after_qr()
            return

        elapsed = time.time() - self.qr_search_start_time

        # 如果设置为“电脑一解出来就走”，这里会提前采用；
        # 默认我们转满 20 秒后再用结果，满足你说的“扫码点转 20s”。
        if (not self.pc_qr_decide_after_full_search) and self.use_pc_qr_result_now(reason='early_pc_result'):
            return

        # 搜索超时：优先采用电脑 20 秒内识别到的结果；没有结果再默认。
        if elapsed > self.qr_search_timeout:
            self.publish_stop()

            if self.use_pc_qr_result_now(reason='pc_result_after_full_20s_search'):
                return

            self.qr_locked = True
            self.mission_direction = self.default_direction
            self.get_logger().warn(
                f'QR_SEARCH timeout after {self.qr_search_timeout:.1f}s. '
                f'No PC QR result. Use manual default direction={self.default_direction}({direction_cn(self.default_direction)}).'
            )
            self.print_qr_terminal_result(
                decoded=False,
                source='QR_SEARCH_TIMEOUT_DEFAULT_FINAL',
                raw_text='NO_QR_DECODED',
                direction=self.default_direction,
                method='default_after_timeout_no_pc_result',
            )
            self.announce_direction(self.default_direction, reason='qr_default_after_search')
            self.start_mission_route_after_qr()
            return

        cmd = Twist()

        # 第一阶段：慢慢后退 + 轻微向左，给二维码留出距离。
        if elapsed < self.qr_search_backup_time:
            cmd.linear.x = self.qr_search_backup_vx
            cmd.angular.z = self.qr_search_backup_wz
            phase = 'BACKUP_AND_LEFT_FOR_FOCUS'

        else:
            # 第二阶段：慢速大角度摆头。
            # 一个周期：左大扫 -> 停一下 -> 右小回 -> 停一下。
            # 左扫更久，右回更短，所以总视野会更偏左，满足“向左边转的大一点”。
            t = elapsed - self.qr_search_backup_time
            cycle = (
                self.qr_search_left_time
                + self.qr_search_hold_time
                + self.qr_search_right_time
                + self.qr_search_hold_time
            )
            c = t % cycle

            cmd.linear.x = self.qr_search_vx

            if c < self.qr_search_left_time:
                cmd.angular.z = self.qr_search_left_wz
                phase = 'SLOW_BIG_LEFT_SWEEP'
            elif c < self.qr_search_left_time + self.qr_search_hold_time:
                cmd.angular.z = 0.0
                phase = 'LEFT_HOLD_STABLE_FRAME'
            elif c < self.qr_search_left_time + self.qr_search_hold_time + self.qr_search_right_time:
                cmd.angular.z = self.qr_search_right_wz
                phase = 'SLOW_SMALL_RIGHT_RETURN'
            else:
                cmd.angular.z = 0.0
                phase = 'RIGHT_HOLD_STABLE_FRAME'

        self.cmd_pub.publish(cmd)

        self.get_logger().warn(
            f'QR_SEARCH {phase} elapsed={elapsed:.1f}/{self.qr_search_timeout:.1f}s '
            f'vx={cmd.linear.x:.3f} wz={cmd.angular.z:.2f}',
            throttle_duration_sec=0.5,
        )

    def on_goal_reached(self, name):
        if self.state == 'NAV_TO_QR' and name == 'QR_POINT':
            # 到扫码点还没扫到：先不要立刻默认，进入 QR_SEARCH 原地大幅搜索几秒。
            if not self.qr_locked:
                self.start_qr_search(reason='QR_POINT reached but no QR yet')
                return

            self.start_mission_route_after_qr()
            return

        if self.state == 'NAV_QUEUE':
            if self.nav_queue and self.nav_queue[0] == name:
                self.nav_queue.pop(0)
            else:
                self.get_logger().warn(f'Nav queue mismatch reached={name}, queue={self.nav_queue}')
                if self.nav_queue:
                    self.nav_queue.pop(0)

            if self.nav_queue:
                self.send_goal(self.nav_queue[0])
            else:
                next_state = self.after_nav_state
                self.after_nav_state = None
                self.state = next_state
                self.on_enter_state(next_state)
            return

    def on_goal_failed(self, name, reason):
        self.get_logger().error(f'on_goal_failed name={name}, reason={reason}, state={self.state}')

        if self.state == 'NAV_TO_QR':
            # 去 QR_POINT 失败/超时：不播报“扫码失败”，播报手动设置的默认方向，然后继续任务。
            if not self.qr_locked:
                self.qr_locked = True
                self.mission_direction = self.default_direction
                self.get_logger().warn(
                    f'QR_POINT failed/timeout. Use manual default direction={self.default_direction}.'
                )
                self.print_qr_terminal_result(
                    decoded=False,
                    source='QR_POINT_NAV_FAIL_DEFAULT_FINAL',
                    raw_text='NO_QR_DECODED',
                    direction=self.default_direction,
                    method='default_after_nav_fail',
                )
                self.announce_direction(self.default_direction, reason='qr_default_after_nav_fail')
            self.start_mission_route_after_qr()
            return

        if self.state == 'NAV_QUEUE':
            count = self.fail_counts.get(name, 0) + 1
            self.fail_counts[name] = count

            # 关键：一定要回出发区。P_RETURN_NAV 不能一次失败就 COMPLETE。
            if name == 'P_RETURN_NAV':
                if count <= 2:
                    self.get_logger().warn(f'P_RETURN_NAV failed/timeout count={count}. Retry P_RETURN_NAV.')
                    self.send_goal('P_RETURN_NAV')
                    return
                self.get_logger().warn('P_RETURN_NAV failed too many times. Try P_START fallback.')
                self.start_nav_queue(['P_START'], after_state='COMPLETE')
                return

            # 出通道点也很关键，先重试一次，别 12 秒一到就跳回家。
            if name == 'C_EXIT_CHANNEL' and count <= 1:
                self.get_logger().warn('C_EXIT_CHANNEL failed/timeout. Retry once before return.')
                self.send_goal('C_EXIT_CHANNEL')
                return

            # C 区角点如果失败，重试一次；再次失败再跳过，避免卡死。
            if name.startswith('C_') and count <= 1:
                self.get_logger().warn(f'{name} failed/timeout count={count}. Retry once.')
                self.send_goal(name)
                return

            # 尽量从队列里弹掉当前失败点。
            if self.nav_queue and self.nav_queue[0] == name:
                self.nav_queue.pop(0)
            elif self.nav_queue:
                self.get_logger().warn(f'Queue mismatch on failure. Drop first waypoint: {self.nav_queue[0]}')
                self.nav_queue.pop(0)

            if self.nav_queue:
                self.get_logger().warn(f'Skip failed waypoint {name}, go next: {self.nav_queue[0]}')
                self.send_goal(self.nav_queue[0])
            else:
                # 如果路线跑空但还没回 P_RETURN_NAV，则补一次回家。
                self.get_logger().warn('Nav queue empty after failure. Force return to P_RETURN_NAV.')
                self.start_nav_queue(['P_RETURN_NAV'], after_state='COMPLETE')
            return

        self.state = 'COMPLETE'
        self.on_enter_state('COMPLETE')

    def check_goal_timeout(self):
        if not self.goal_active or self.current_goal_name is None:
            return
        timeout = self.goal_timeout.get(self.current_goal_name, self.goal_timeout['default'])
        elapsed = time.time() - self.goal_send_time
        if elapsed > timeout:
            name = self.current_goal_name
            self.cancel_active_goal(reason=f'timeout {elapsed:.1f}s > {timeout:.1f}s')
            self.on_goal_failed(name, 'timeout')

    # ============================================================
    # 状态入口与主循环
    # ============================================================
    def on_enter_state(self, state):
        if state == 'COMPLETE':
            self.publish_stop()
            self.play_or_speak('mission_complete', '任务完成', 'mission complete')
            self.get_logger().warn('Mission COMPLETE')
            return

    def control_loop(self):
        if self.state == 'WAIT_NAV2':
            if not self.nav_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...', throttle_duration_sec=1.0)
                return

            self.nav2_ready = True
            if self.wait_for_manual_start:
                self.get_logger().warn(
                    'Nav2 action server ready. WAIT_START: press SPACE+ENTER or ENTER in this terminal to start.',
                    throttle_duration_sec=1.5,
                )
                self.state = 'WAIT_START'
                return

            self.get_logger().warn('Nav2 action server ready. Send QR_POINT, scan QR continuously on the way.')
            self.state = 'NAV_TO_QR'
            self.send_goal('QR_POINT')
            return

        if self.state == 'WAIT_START':
            if self.manual_start_received:
                self.start_first_goal_after_gate()
                return
            self.publish_stop()
            self.get_logger().warn(
                'WAIT_START: node is warm. Press SPACE+ENTER or ENTER to start mission.',
                throttle_duration_sec=1.5,
            )
            return

        if self.state == 'NAV_TO_QR':
            # 边走边看码：Nav2 负责往 QR_POINT 走，本节点的 image_callback 同时持续识别二维码。
            # 一旦扫到，lock_qr_direction() 会立刻 cancel QR_POINT 并进入任务路线。
            self.get_logger().warn(
                'NAV_TO_QR: moving to QR_POINT and scanning QR continuously on the way.',
                throttle_duration_sec=1.5,
            )
            self.check_goal_timeout()
            return

        if self.state == 'NAV_QUEUE':
            self.check_goal_timeout()
            return

        if self.state == 'QR_SEARCH':
            self.handle_qr_search()
            return

        if self.state == 'COMPLETE':
            self.publish_stop()
            self.get_logger().warn('Mission complete.', throttle_duration_sec=2.0)
            return

        self.get_logger().warn(f'Unknown state={self.state}. Stop.')
        self.publish_stop()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pc_qr_stop_event.set()
        except Exception:
            pass
        try:
            node.cancel_active_goal('shutdown')
            node.publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 

if __name__ == '__main__':
    main()
