#!/usr/bin/env python3
"""
相机驱动接入 — 用 usb_cam 把两路相机接进感知链路（P2-14 的剩下那一半）

═══════════════════════════════════════════════════════════════════════════════
为什么需要这个文件
═══════════════════════════════════════════════════════════════════════════════
在它之前，**全仓库没有任何东西发布** `/camera_wide/image_raw` 与
`/camera_tele/image_raw` —— 唯一的仓库内生产者是 Gazebo。后果是链路缺输入：

  · `mustika_pillar_check`（**250 分**）永远报 UNKNOWN（它要的是相机检出的球）
  · 天空方块判色（**40/80 分**的归属）拿不到任何图像

而话题名在 `src/camera/image_preprocessor.cpp:126,130` 里是**硬编码字面量**
（不是参数）⇒ 想让节点读到相机，**只能靠 launch 的 `remap`**，改 yaml 没用。

═══════════════════════════════════════════════════════════════════════════════
⚠️⚠️ 三个实测出来的坑（2026-09-25 在本机验证，不是从文档抄的）
═══════════════════════════════════════════════════════════════════════════════

① **曝光单位差 100 倍** —— 这是最容易「一调就全白」的地方。
   `config/camera_hardware.yaml` 写的是 `exposure_time: 5000`，注释标 **μs**（=5ms）。
   但 V4L2 的 `exposure_time_absolute`（usb_cam 的 `exposure` 参数直接透传它）
   单位是 **100μs** ⇒ 想表达 5ms 必须传 **50**。
   ⚠️ 把 5000 原样传进来 = **500ms 曝光** ⇒ 画面过曝成一片白
   （本机第一次抓图正是这个现象：对着灯、整幅纯白）。
   ⇒ 本文件的 `exposure` 默认 **50**，**单位是 100μs**。

② **必须用 MJPG**。两路相机都枚举在 **480 Mbps** 总线上（`speed=480Mbps`），
   而 1280×720 的 **YUYV 未压缩**在格式表里只列到 **10 fps**。
   本机实测持续帧率：**720p MJPG = 29.1 fps**、**1080p MJPG = 20.1 fps**
   ⇒ 换 YUYV 掉到 10fps，换 1080p 掉到 20fps。**这场比赛要的是 29fps。**

③ **`exposure_dynamic_framerate` 会让帧率随曝光时间掉**。本机 `/dev/video2`
   默认 **开**（值=1），语义是「暗光下拉长曝光、自动掉帧」。
   竞赛要的是**稳定帧率**（§零：延迟就是分数）⇒ 本文件起驱动前显式关掉它。
   ⚠️ 它不是 usb_cam 的参数，是 V4L2 控制项，所以只能用 `SetV4l2Controls` 那类外部手段
      —— 本文件用 `ExecuteProcess` 调 `v4l2-ctl` 关（见 `_disable_dynamic_framerate`）。

═══════════════════════════════════════════════════════════════════════════════
⚠️ 与 `config/camera_hardware.yaml` 的**口径差异**（有意不改那个文件）
═══════════════════════════════════════════════════════════════════════════════
那个文件是「**驱动层规格**」（`scripts/calibrate_camera.py` 在读它），本文件的职责是
「**怎么把实际的相机起起来**」。几处对不上，记在这里免得下次又踩：

| `camera_hardware.yaml` | 实际 UVC 能力 | 本文件怎么做 |
|---|---|---|
| `pixel_format: "BGR8"` | usb_cam 收 `mjpeg2rgb` / `yuyv2rgb` / `raw_mjpeg` | 传 **`mjpeg2rgb`**（解出 BGR8），语义等价 |
| `exposure_time: 5000` (μs) | `exposure` 单位是 **100μs** | 传 **50**（见坑 ①） |
| `white_balance_mode: "manual"` + `white_balance_red/blue: 1.5` | **UVC 没有 R/B 增益**，只有 `white_balance_temperature`(K) | 传 **`auto_white_balance: true`** |
| `gain: 0.0` | `gain` 是 int 0..63 / 0..100 | 传 **0** |

⚠️ **`white_balance_red/blue` 这条要单独留意**：如果判色（红/蓝天空方块）受白平衡影响，
UVC 能给的只有色温，**没法像厂商 SDK 那样锁死 R/B 增益** ⇒ 判色阈值必须按
**这台机器实际出的图**标（`scripts/hsv_tuner.py`），不能照规格纸面值。

═══════════════════════════════════════════════════════════════════════════════
⚠️ `video_device` 的默认值是**猜的** —— 装反了就对调
═══════════════════════════════════════════════════════════════════════════════
本机 2026-09-25 实测：
  · `/dev/video0` = **icSpring** (32e6:9211)，支持 1280×720
  · `/dev/video2` = **HD USB Camera** (05a3:9230)，支持 1920×1080
**哪台装的是广角、哪台是长焦，从设备串里看不出来**（那取决于拧的镜头）。
默认 video0→wide、video2→tele；**装反了对调两个参数**：
  ros2 launch br_perception camera_bringup.launch.py wide_device:=/dev/video2 tele_device:=/dev/video0
⚠️ 装反的后果是**外参全错**（两路内参 fx 差约 3 倍：wide 800 / tele 2380）。

⚠️ 另外 `/dev/videoN` 编号**不保证跨次开机稳定** —— 插拔或换口都会变。
   长期做法是按 `by-id` 或序列号绑定；本轮先用编号，起不来时按下面的提示换。

═══════════════════════════════════════════════════════════════════════════════
用法
═══════════════════════════════════════════════════════════════════════════════
  source /opt/ros/humble/setup.bash
  source ~/point_ws/install/setup.bash        # ⚠️ 不是 ~/ws_livox（那个路径不存在）

  ros2 launch br_perception camera_bringup.launch.py                    # 两路
  ros2 launch br_perception camera_bringup.launch.py enable_tele:=false  # 只一路

  # 唯一的判据 —— 不是"节点活着"，而是有数：
  ros2 topic hz /camera_wide/image_raw

⚠️ **本文件尚未在真机上跑通**（参数按 2026-09-25 实测值写，但"起得来 + 帧率达标"
   要起一次才知道）。验收判据与结果见 `doc/模块进度.md` §八。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _mvs_node(role, serial, frame_id, condition=None):
    """一路 —— **海康 GigE 相机**（走 MVS SDK）。

    ⚠️ **为什么要单独一个**：海康 `MV-CS020-10GC` 是 GigE，**不产生 `/dev/video*`**
       （`v4l2-ctl --list-devices` 里没有它）⇒ `v4l2_camera` 那条 V4L2 路走不通。
    ⚠️ **`auto_exposure` / `auto_white_balance` 是字符串**（MVS 的枚举值 `Off`/`Once`/
       `Continuous`）。在 launch 里传 Python 字符串没问题；但用命令行
       `-p auto_exposure:=Off` 时 **`Off` 会被 YAML 解析成 bool** ⇒ 类型不匹配
       （节点现在会点名报错并沿用默认值，不会崩）。命令行要写 `'Off'`。
    ⚠️ 曝光单位是 **µs**（MVS 口径），与 `v4l2_camera` 的 100µs **不同**，别混。
    """
    kwargs = {'condition': condition} if condition is not None else {}
    return Node(
        package='br_perception',
        executable='mvs_camera',
        name='mvs_camera',
        namespace=f'camera_{role}',
        output='screen',
        parameters=[{
            'serial_number': serial,          # ⚠️ 多台时靠它选，别靠枚举顺序
            'frame_id': frame_id,
            'camera_name': f'camera_{role}',
            'exposure_us': 5000.0,            # MVS 单位 µs
            'gain_db': 0.0,
            'auto_exposure': 'Off',           # 赛场灯会变，自动曝光会漂
            'auto_white_balance': 'Off',      # ⚠️ 自动白平衡会动判色阈值
            'pixel_format': 'BayerRG8',       # 这台不支持机内转 BGR8（实测）
            'debayer': 'RG',                  # ⚠️ 相位已实测验证，改错会红蓝互换
            'timestamp_mode': 'capture',
            # ⚠️ 限帧 = **直接降发热**。本机实测它自由跑 44.8fps，而比赛只要 30
            #    ⇒ 不限就是白烧电变热（用户 2026-09-25 反馈"太烫"）。
            #    代价：帧周期 22→33ms，相机源年龄 +11ms（31→42ms），总延迟仍在 warn 内。
            'frame_rate': 30.0,
        }],
        **kwargs,
    )


def _v4l2_node(role, device, frame_id, condition=None):
    """一路相机 —— **走本仓库自己的 `v4l2_camera`**（`driver:=v4l2`，默认）。

    ⚠️ **为什么不再是 `usb_cam`**（2026-09-25 实测）：`usb_cam` 写进 `header.stamp`
       的时间戳比发布时刻**老约 600~800 ms**。同一台相机、同一个订阅端探针对照：
         · `usb_cam`                 → 810 ms
         · 本节点的 `v4l2_camera`    → **47 ms**
       ⇒ 端到端 `frame_age_sec` 从 **0.83~0.92 s** 降到 **0.066~0.069 s**。
       根因是它的时间戳换算，不是画面真的旧。详见 `doc/模块进度.md` §八 9.15。

    ⚠️ **话题靠 `namespace` 摆位**，不用 remap —— `v4l2_camera` 发的是相对名
       `image_raw` / `camera_info` ⇒ 命名空间一给，就落在 `/camera_wide/image_raw`。
       （`usb_cam` 发在根命名空间，只能 remap；换节点后这点更干净。）

    ⚠️ `frame_id` 必须与 `config/camera_params.yaml:207` 的
       `camera_frames: ["camera_wide_frame", "camera_tele_frame"]` **逐字一致** ——
       `sky_block_detector` 拿它做 TF lookup（投影雷达建筑位到图像 ROI），
       名字对不上 = TF 失败 = 判色那条路整个不工作。
    """
    kwargs = {'condition': condition} if condition is not None else {}
    return Node(
        package='br_perception',
        executable='v4l2_camera',
        name='v4l2_camera',
        namespace=f'camera_{role}',          # → /camera_wide/image_raw
        output='screen',
        parameters=[{
            'video_device': device,
            'framerate': LaunchConfiguration('framerate'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'pixel_format': 'MJPG',          # 见坑 ②：USB2 上必须 MJPG
            'frame_id': frame_id,
            'camera_name': f'camera_{role}',
            # 时间戳默认取自驱动缓冲（**每帧重算时钟偏移**）——
            # 只有排查时才切 "publish"，那会把真实采集延迟藏起来。
            'timestamp_mode': 'capture',
            'auto_exposure': False,          # 赛场灯会变，自动曝光会漂
            'exposure_absolute': LaunchConfiguration('exposure'),  # ⚠️ 单位 100μs
            'gain': 0,
            'buffer_count': 4,
        }],
        **kwargs,
    )


def generate_launch_description():
    decls = [
        DeclareLaunchArgument(
            'wide_device', default_value='/dev/video0',
            description='广角相机设备节点（⚠️ 默认值是猜的，装反了就对调）'),
        DeclareLaunchArgument(
            'tele_serial', default_value='DB2615603',
            description='海康 GigE 相机的**序列号**（不是 /dev/videoN —— GigE 没有设备节点）。'
                        '⚠️ 多台海康时必须给序列号，靠枚举顺序会选错 ⇒ 外参全错。'
                        '换相机就换成 `MvImport` 枚举出来的那个值（本机实测值就是默认值）'),
        DeclareLaunchArgument(
            'enable_tele', default_value='true',
            description='是否起长焦那一路（只有一台相机时置 false）'),
        DeclareLaunchArgument(
            'image_width', default_value='1280',
            description='⚠️ 必须与 config/camera_hardware.yaml 的 resolution 一致'),
        DeclareLaunchArgument(
            'image_height', default_value='720',
            description='⚠️ 1080p 在本机 USB2 上只有 20fps，见坑 ②'),
        DeclareLaunchArgument(
            'framerate', default_value='30.0',
            description='标称帧率（实测：720p MJPG 约 29fps，受 USB 带宽限制）'),
        DeclareLaunchArgument(
            'exposure', default_value='50',
            description='⚠️ 单位 100μs ⇒ 50 = 5ms，与 camera_hardware.yaml 的 '
                        'exposure_time:5000(μs) 等价。**别传 5000**，那是 500ms ⇒ 全白'),
    ]

    # ⚠️ 这条提示有用：没接相机时 usb_cam **起得来但一直刷错误**，
    #    没有这条输出，现场容易误以为"launch 坏了"。
    hint = LogInfo(msg=[
        '\n[相机驱动] 判据不是"节点活着"，而是 `ros2 topic hz /camera_wide/image_raw` 有数。\n',
        '  刷 "Cannot open" ⇒ 设备号不对：先 `v4l2-ctl --list-devices`，再覆盖 wide_device/tele_device。\n',
        '  帧率低 ⇒ 检查是否插在 USB3 口、以及是否用了 MJPG（见本文件头部坑 ②）。\n',
    ])

    return LaunchDescription(decls + [
        hint,
        # ⚠️ 不再需要 `v4l2-ctl` 外部关 `exposure_dynamic_framerate` ——
        #    `v4l2_camera` 在设备打开时就把 `V4L2_CID_EXPOSURE_AUTO_PRIORITY=0` 设掉了
        #    （那才是"暗光下拉长曝光、自动掉帧"的正确解法，且在节点内、可复现）。
        #
        # ── 台位分工（2026-09-25 定）──
        #   wide = JR（UVC，/dev/video*）→ 近场：穆斯蒂卡球 250 分 / 建筑位 / 柱子
        #   tele = 海康 MV-CS020-10GC（GigE）→ 远场：天空方块塔顶
        # ⚠️ 两者**驱动不同、契约相同**（同 QoS / 同 frame_id 口径 / 同诊断），下游不用改。
        _v4l2_node('wide', LaunchConfiguration('wide_device'), 'camera_wide_frame'),
        _mvs_node('tele', LaunchConfiguration('tele_serial'), 'camera_tele_frame',
                  condition=IfCondition(LaunchConfiguration('enable_tele'))),
    ])
