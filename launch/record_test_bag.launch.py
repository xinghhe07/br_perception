#!/usr/bin/env python3
"""
测试数据录制 — 任务_v1.7 §10.5 / §15.4 第 3 层（ROS Bag 回放）

**用途**：把一次真实/仿真的传感器输入录下来，之后**无实物反复回放联调** ——
这是硬件不在手边时唯一能重跑整条感知链的手段（§15.4 的"无实物测试"第 3 层）。

═══════════════════════════════════════════════════════════════════════════════
录什么：**默认只录"根数据"**（传感器原始输入），不录感知输出
═══════════════════════════════════════════════════════════════════════════════
  理由：根数据在手，**整条链就能重跑**（含预处理、letterbox、阈值调整）；
        反过来只录输出的话，改了任何一环都没法重新验证，bag 就废了。

  根数据（默认）:
    /livox/lidar/pointcloud        雷达 → 感知链的输入
    /livox/imu                     IMU（Point-LIO 要用）
    /camera_wide/image_raw         工业相机 宽
    /camera_tele/image_raw         工业相机 长焦
    /camera/depth/image_rect_raw   D435i 深度
    /tf  /tf_static                坐标系（回放时 TF 也要在）

  ⚠️ `/livox/lidar`（CustomMsg，Point-LIO 的根）**默认不录** —— 它与
     `/livox/lidar/pointcloud` 是**同一份数据的两条话题**（后者由桥接节点转换），
     两个都录等于**点云占双份空间**。要连 Point-LIO 一起重跑时加 `raw_livox:=true`。
  ⚠️ 感知**输出**默认不录（`perception_out:=false`）—— 回放时它们会被重新算出来。
     只有"想事后对照两次运行的差异"时才需要录，见下。

═══════════════════════════════════════════════════════════════════════════════
⚠️ 磁盘（无风扇迷你主机，空间有限）
═══════════════════════════════════════════════════════════════════════════════
  两路 1280×720 图像 @30Hz 是很贵的。粗算：一张 720p 原图 ~0.4 MB，
  30Hz × 2 路 ≈ **24 MB/s ≈ 1.4 GB/分钟**（未压缩，rosbag2 默认不压缩）。
  跑 3 分钟比赛就是 4 GB+。所以：
    · `bag_dir` 默认落在 `~/br_perception_bags`，**别录到系统盘**
    · `max_bag_size_mb` 默认 4096，超了自动**分卷**（rosbag2 的 `-b`）——
      宁可多几个文件，也别把一个 bag 写到把盘撑满
    · ⚠️ 录之前先 `df -h` 看一眼。录一半写满盘，**bag 是坏的且现场不知道**

═══════════════════════════════════════════════════════════════════════════════
⚠️ 为什么本文件**不起雷达驱动**（任务书 §10.5 写的是"启动所有传感器驱动"）
═══════════════════════════════════════════════════════════════════════════════
  雷达驱动的启动代码在**三个** launch 里**各有一份副本**
  （`bringup_full` / `bringup_lidar_only` / `pointlio_mapping`，每份都带一个
   `_resolve_launch` —— livox 的 launch 在 `launch/` 还是 `launch_ROS2/` 下因版本而异，
   所以那个 helper 没法简单删掉）。台账把这笔重复记在案
  （`doc/模块进度.md` §六），且已定"等联调跑通再抽 `launch/common.py`"。

  ⇒ 本文件**不再抄第 4 份**。需要驱动时先另开一个终端起一个 bringup 即可：
       ros2 launch br_perception bringup_lidar_only.launch.py

  ⚠️ 这也意味着**录制与驱动是两件事，别指望一条命令搞定**。好处是：
     驱动用哪个 launch（仿真 / 实机 / 只雷达）由你定，本文件不替你选。

前置:
  source /opt/ros/humble/setup.bash
  source ~/ws_livox/install/setup.bash
  source ~/br_perception/install/setup.bash

用法:
  # 只录（传感器驱动已经在跑）
  ros2 launch br_perception record_test_bag.launch.py venue:=lab

  # 连桥接 + 静态 TF 一起起（⚠️ **雷达驱动不在本文件里**，要另起，见下）
  ros2 launch br_perception record_test_bag.launch.py venue:=lab start_drivers:=true

  # 连 Point-LIO 的根一起录（要重跑里程计时）
  ros2 launch br_perception record_test_bag.launch.py venue:=lab raw_livox:=true

  # 连感知输出一起录（事后对照两次运行的差异）
  ros2 launch br_perception record_test_bag.launch.py venue:=lab perception_out:=true

  # 录完怎么回放
  python3 scripts/replay_bag_debug.py ~/br_perception_bags/20260923_1430_lab
"""

import os
import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, Node,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('br_perception')

    # ── launch 参数 ──
    venue_arg = DeclareLaunchArgument(
        'venue', default_value='lab',
        description='场地标识，进 bag 目录名（如 lab / field / home）。'
                    '录不同场地时分得开 —— 现场最容易搞混的就是"这个 bag 是哪录的"')
    bag_dir_arg = DeclareLaunchArgument(
        'bag_dir', default_value=os.path.expanduser('~/br_perception_bags'),
        description='bag 根目录。⚠️ 别放系统盘，见文件头的磁盘说明')
    start_drivers_arg = DeclareLaunchArgument(
        'start_drivers', default_value='false', choices=['true', 'false'],
        description='起桥接 livox_custom_to_pcl2 + 静态 TF。'
                    '⚠️ **不含雷达驱动本身**（见文件头"为什么不起雷达驱动"）。'
                    'false = 假定这些已经在别的终端跑着')
    raw_livox_arg = DeclareLaunchArgument(
        'raw_livox', default_value='false', choices=['true', 'false'],
        description='额外录 /livox/lidar (CustomMsg)，为的是**回放时能重跑 Point-LIO**。'
                    '⚠️ 与 pointcloud 是同一份数据的两条话题，点了点云占双份空间')
    perception_out_arg = DeclareLaunchArgument(
        'perception_out', default_value='false', choices=['true', 'false'],
        description='额外录**节点输出**（感知结果 + 里程计）。回放时会被重算，'
                    '通常不需要；只有"事后对照两次运行的差异"时才录')
    max_size_arg = DeclareLaunchArgument(
        'max_bag_size_mb', default_value='4096',
        description='单个 bag 文件上限 [MB]，超了自动分卷。防"录到写满盘"')

    venue = LaunchConfiguration('venue')
    bag_dir = LaunchConfiguration('bag_dir')
    raw_livox = LaunchConfiguration('raw_livox')
    perception_out = LaunchConfiguration('perception_out')
    max_size = LaunchConfiguration('max_bag_size_mb')

    # ═══════════════════════════════════════════════════════════════════════
    # 话题白名单 —— **一处定义**，避免"驱动录了、回放时发现少一路"
    # ═══════════════════════════════════════════════════════════════════════
    # ⚠️ 这些话题名是从各节点的 create_subscription/create_publisher **逐个核出来的**
    #    （2026-09-23），不是照文档抄的 —— 文档里的话题名有过期风险。
    #    改话题名时**这里要跟着改**，否则症状是"bag 录出来是空的但没报错"。
    ROOT_TOPICS = [
        '/livox/lidar/pointcloud',       # 雷达 → 感知链输入
        '/livox/imu',                    # IMU（Point-LIO）
        '/camera_wide/image_raw',        # 工业相机 宽
        '/camera_tele/image_raw',        # 工业相机 长焦
        '/camera/depth/image_rect_raw',  # D435i 深度
        '/tf',
        '/tf_static',
    ]
    # 只在 raw_livox:=true 时加 —— 与 pointcloud 同源，双录等于点云占两份
    # （要连 Point-LIO 一起重跑时才需要：Point-LIO 吃的是 CustomMsg）
    OPT_RAW_LIVOX = ['/livox/lidar']
    # 节点**输出** —— 回放时会被重算，只有"对照两次运行差异"才需要。
    # ⚠️ `/aft_mapped_to_init` 是 Point-LIO 的**输出**（里程计），不是传感器输入 ——
    #    它属于这一类，别混进根数据里（回放时它会重新算出来）。
    OPT_PERCEPTION_OUT = [
        '/perception/lidar/result',
        '/perception/camera/frame',
        '/perception/output',
        '/system/degradation_state',
        '/aft_mapped_to_init',
    ]

    def _make_bag_process(context):
        """OpaqueFunction: 话题列表要读 launch 参数后才能定，生成期读不到。"""
        lc = context.launch_configurations

        def on(flag):
            return str(lc.get(flag, 'false')).lower() in ('true', '1', 'yes')

        topics = list(ROOT_TOPICS)
        if on('raw_livox'):
            topics += OPT_RAW_LIVOX
        if on('perception_out'):
            topics += OPT_PERCEPTION_OUT

        # ── bag 目录名：日期_时间_场地 ──
        # ⚠️ 时间戳在**生成期**取一次就固定，不用 LaunchConfiguration 拼 ——
        #    后者每次引用都会重新求值，可能得到两个不同的时刻。
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        out_dir = os.path.join(os.path.expanduser(lc.get('bag_dir', '~/br_perception_bags')),
                               f'{stamp}_{lc.get("venue", "lab")}')

        # ⚠️ `-b` 的单位是**字节**（不是 MB），而且 rosbag2 会在超出时**分卷改名**。
        size_bytes = int(float(lc.get('max_bag_size_mb', '4096')) * 1024 * 1024)

        cmd = ['ros2', 'bag', 'record',
               '-o', out_dir,
               '-b', str(size_bytes),
               # ⚠️ 不写 `--compression-mode`：默认不压缩，CPU 全留给感知。
               #    无风扇迷你主机上，边录边压很容易把 CPU 吃满、反过来把要录的帧丢掉。
               ] + topics

        return [
            LogInfo(msg=f'[record_test_bag] → {out_dir}'),
            LogInfo(msg='  录制话题: ' + ' '.join(topics)),
            LogInfo(msg='  ⚠️ 先 df -h 看一眼！两路 720p@30Hz ≈ 1.4 GB/分钟'),
            ExecuteProcess(cmd=cmd, output='screen'),
        ]

    # ═══════════════════════════════════════════════════════════════════════
    # 可选的驱动（雷达 + 桥接 + 静态 TF）—— 不含 Point-LIO
    #   ⚠️ 要录里程计就另外起 `bringup_lidar_only` 或 `pointlio_mapping`，
    #      本文件**刻意不重复实现**那套（Point-LIO 的启动参数有它自己的一份事实源）
    # ═══════════════════════════════════════════════════════════════════════
    drivers = [
        Node(package='br_perception', executable='livox_custom_to_pcl2',
             name='livox_custom_to_pcl2', output='screen',
             parameters=[{'input_topic': '/livox/lidar',
                          'output_topic': '/livox/lidar/pointcloud',
                          'frame_id': 'livox_frame'}],
             condition=IfCondition(LaunchConfiguration('start_drivers'))),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_base_to_livox',
             arguments=['--x', '0.0', '--y', '0.0', '--z', '0.25',   # ⚠ 占位
                        '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                        '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
             condition=IfCondition(LaunchConfiguration('start_drivers'))),
    ]

    return LaunchDescription([
        venue_arg,
        bag_dir_arg,
        start_drivers_arg,
        raw_livox_arg,
        perception_out_arg,
        max_size_arg,
        LogInfo(msg='[br_perception] 测试数据录制 — §10.5 / §15.4 第 3 层'),
        LogInfo(msg='  ⚠️ **雷达驱动不在本文件里起**（start_drivers 只起桥接与静态TF）——'
                    '任务书 §10.5 要的"启动所有传感器驱动"由调用者负责：'
                    '先另开终端 `ros2 launch br_perception bringup_lidar_only.launch.py`，'
                    '再跑本文件。理由见文件头"为什么不起雷达驱动"。'),
    ] + drivers + [
        OpaqueFunction(function=_make_bag_process),
    ])
