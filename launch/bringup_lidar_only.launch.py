#!/usr/bin/env python3
"""
降级模式启动 — 只用雷达 (任务_v1.7 §10.2, 相机故障时使用)

启动内容:
  1. Livox Mid-360 雷达驱动            (CustomMsg 输出 — Point-LIO 要求)
  2. Point-LIO 里程计节点               (纯里程计, §十.2)
  3. Livox CustomMsg → PointCloud2 桥接 (感知管线要 PointCloud2)
  4. 静态外参 TF (base_link ← livox_frame)  ⚠ 占位值, 待标定
  5. 雷达感知管线 (br_perception 主节点)
  6. 通信节点 — ⬜ 待实现; 实现后天空方块颜色须标记为 UNKNOWN

前置:
  source /opt/ros/humble/setup.bash
  source ~/point_ws/install/setup.bash
  source ~/lio_ws/install/setup.bash
  source ~/br_perception/install/setup.bash

用法:
  ros2 launch br_perception bringup_lidar_only.launch.py
  ros2 launch br_perception bringup_lidar_only.launch.py lio:=false            # 不跑里程计
  ros2 launch br_perception bringup_lidar_only.launch.py perception_rviz:=false

  # 仿真 (与 bringup_gazebo 配套: 点云由它桥接, 里程计它也有一个)
  ros2 launch br_perception bringup_lidar_only.launch.py        livox:=false lio:=false use_sim_time:=true

⚠️⚠️ **本 launch 不起 `perception_pipeline` ⇒ `world` 帧没人广播** ⇒ 雷达节点的
    `cloud_to_world` 会失败, **10 个建筑位全报 UNKNOWN**（会打一条点名的 WARN）。
    这在**台架**上是预期的（场地常量在台子上没有对应物）；但要在台架上验建筑位,
    得另外起 `perception_pipeline`（`bringup_full` 里就有）, 或临时 `world_frame:""`。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_launch(share_dir, launch_file):
    """在 launch/ 与 launch_ROS2/ (livox_ros_driver2 用后者) 下找 launch 文件"""
    for sub in ('launch', 'launch_ROS2', ''):
        candidate = os.path.join(share_dir, sub, launch_file) if sub \
            else os.path.join(share_dir, launch_file)
        if os.path.exists(candidate):
            return candidate
    raise RuntimeError(
        f"在 {share_dir} 下找不到 {launch_file} (已试 launch/ 与 launch_ROS2/)")


def _include(pkg_name, launch_file, launch_arguments=None, source_hint="", condition=None):
    """外部包 launch 的安全 include — 包不存在时给出可读的中文提示"""
    try:
        share = get_package_share_directory(pkg_name)
    except Exception as exc:  # PackageNotFoundError
        raise RuntimeError(
            f"找不到包 '{pkg_name}' — 请先 source 对应工作区。{source_hint}"
        ) from exc
    kwargs = {'condition': condition} if condition is not None else {}
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_resolve_launch(share, launch_file)),
        launch_arguments=(launch_arguments or {}).items(),
        **kwargs,
    )


def generate_launch_description():
    share_dir = get_package_share_directory('br_perception')

    # ⚠ 参数名不用 'rviz': include 的 launch_arguments 会写进共享 launch context,
    #   给 point_lio 传 rviz:=false 会把同名参数一起污染, 导致本文件的 rviz 静默不启动
    rviz_arg = DeclareLaunchArgument(
        'perception_rviz', default_value='true', choices=['true', 'false'],
        description='启动 RViz2 (感知调试视图)')
    lio_arg = DeclareLaunchArgument(
        'lio', default_value='true', choices=['true', 'false'],
        description='启动 Point-LIO 里程计节点 (与雷达驱动分开控制)')
    # ⚠️⚠️ 2026-09-24 新增两个口子 —— 它们让**同一条 launch** 既能跑实机、又能跑仿真：
    #    · `livox`   = 起不起真实雷达驱动。**仿真下必须 false**（仿真里点云由
    #                  `bringup_gazebo` 自己桥接出来）；不起的话点了它只是白开一个
    #                  "找不到设备"的驱动进程，还会与仿真的 point_lio **撞节点名**。
    #    · `use_sim_time` = 用仿真时钟。**仿真下必须 true** —— 否则本节点用系统时间，
    #                  而 TF 是仿真时间戳 ⇒ **查 TF 恒失败 ⇒ 全帧报 UNKNOWN**
    #                  （见 `lidar_perception_node` 的 `cloud_to_world`）。
    livox_arg = DeclareLaunchArgument(
        'livox', default_value='true', choices=['true', 'false'],
        description='启动真实雷达驱动 (livox_ros_driver2)。仿真下必须 false')
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', choices=['true', 'false'],
        description='用仿真时钟 (接 gazebo/bag 回放时设 true)。'
                    '⚠️ 设错一侧 ⇒ TF 时间戳不同源 ⇒ 全帧 UNKNOWN')
    livox = LaunchConfiguration('livox')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('perception_rviz')
    lio = LaunchConfiguration('lio')

    # 1. 雷达驱动 (CustomMsg — Point-LIO 的 lidar_type:1 要求)
    livox_driver = _include(
        'livox_ros_driver2', 'msg_MID360_launch.py',
        source_hint='雷达驱动在 ~/point_ws, 见 ~/livox_lab_up.sh',
        condition=IfCondition(livox))

    # 2. Point-LIO 里程计 (输出 /aft_mapped_to_init → coordinate_transformer §4.1)
    #    关掉它自带的 rviz (上面的参数名注意避开污染)
    point_lio = _include(
        # ⚠️ 2026-09-25 修 `mapping_mid360.launch.py` → `point_lio.launch.py`
        #    （本机 point_lio 只装了后者；理由与症状见 `bringup_full.launch.py` 同一处）。
        #    ⚠️ 这处是**加载期**炸 ⇒ 整个 launch 文件一行都起不来（连 `--show-args` 都失败）。
        'point_lio', 'point_lio.launch.py',
        launch_arguments={'rviz': 'false'},
        source_hint='Point-LIO 在 ~/point_ws/src/point_lio',
        condition=IfCondition(lio))

    # 3. CustomMsg → PointCloud2 桥接 (感知管线订阅 /livox/lidar/pointcloud)
    custom_to_pcl2 = Node(
        package='br_perception', executable='livox_custom_to_pcl2',
        name='livox_custom_to_pcl2', output='screen',
        parameters=[{'input_topic': '/livox/lidar',
                     'output_topic': '/livox/lidar/pointcloud',
                     'frame_id': 'livox_frame'}])

    # 4. 静态外参 TF: base_link ← livox_frame (⚠ 占位, 标定后回填 fusion_params.yaml)
    #    ⚠️ 挂 `livox` 开关下（2026-09-24）：`livox:=false` 时**传感器不是这条 launch 提供的**
    #       —— 仿真里 `bringup_gazebo` 的 `SENSOR_TF` 已经广播了同一条边（z=0.2419）。
    #       两个广播者 = `TF_REPEATED_DATA` + 同一条边两个值（差 8mm），**谁生效不确定**。
    #       这类"同边双广播"是本仓库明令避免的（见 `sky_block_detector.hpp` 那段）。
    static_tf_livox = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_base_to_livox',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.25',      # ⚠ 占位
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
        condition=IfCondition(livox))

    # 5. 雷达感知管线 — 直接起主节点, 不复用 lidar_perception.launch.py
    #    (该 launch 的 map→livox_frame 静态 TF 会与 base_link 树冲突: 同帧双父)
    lidar_pipeline = Node(
        package='br_perception', executable='lidar_perception_node',
        name='lidar_perception_node', output='screen',
        # ⚠️ `debug_flags.yaml` **必须排在最后**（后者覆盖前者）——
        #    本 launch 是台架用，默认仍会拿到它里面关掉的 `debug_enabled`；
        #    要点云就**改 config/debug_flags.yaml**，别改这里（单一开关）。
        parameters=[os.path.join(share_dir, 'config', 'lidar_params.yaml'),
                    os.path.join(share_dir, 'config', 'field_geometry.yaml'),
                    os.path.join(share_dir, 'config', 'debug_flags.yaml'),
                    # ⚠️ `use_sim_time` 必须**排在最后**（后面覆盖前面）——它是节点自己的
                    #    参数，谁都不该盖它。
                    {'use_sim_time': use_sim_time}])

    perception_rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(share_dir, 'config', 'lidar_perception.rviz')],
        condition=IfCondition(rviz))

    # 6. TODO(§7): 通信节点 — 实现后在此启动; 降级模式下天空方块颜色输出 UNKNOWN

    return LaunchDescription([
        rviz_arg,
        livox_arg,
        sim_time_arg,
        lio_arg,
        LogInfo(msg="[br_perception] 降级模式启动 — §10.2 (仅雷达; 相机故障)"),
        static_tf_livox,
        livox_driver,
        point_lio,
        custom_to_pcl2,
        lidar_pipeline,
        perception_rviz_node,
    ])
