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
  source ~/ws_livox/install/setup.bash
  source ~/lio_ws/install/setup.bash
  source ~/br_perception/install/setup.bash

用法:
  ros2 launch br_perception bringup_lidar_only.launch.py
  ros2 launch br_perception bringup_lidar_only.launch.py lio:=false            # 不跑里程计
  ros2 launch br_perception bringup_lidar_only.launch.py perception_rviz:=false
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
    rviz = LaunchConfiguration('perception_rviz')
    lio = LaunchConfiguration('lio')

    # 1. 雷达驱动 (CustomMsg — Point-LIO 的 lidar_type:1 要求)
    livox_driver = _include(
        'livox_ros_driver2', 'msg_MID360_launch.py',
        source_hint='雷达驱动在 ~/ws_livox, 见 ~/livox_lab_up.sh')

    # 2. Point-LIO 里程计 (输出 /aft_mapped_to_init → coordinate_transformer §4.1)
    #    关掉它自带的 rviz (上面的参数名注意避开污染)
    point_lio = _include(
        'point_lio', 'mapping_mid360.launch.py',
        launch_arguments={'rviz': 'false'},
        source_hint='Point-LIO 在 ~/lio_ws (9000_point_lio_ros2_Mid-360)',
        condition=IfCondition(lio))

    # 3. CustomMsg → PointCloud2 桥接 (感知管线订阅 /livox/lidar/pointcloud)
    custom_to_pcl2 = Node(
        package='br_perception', executable='livox_custom_to_pcl2',
        name='livox_custom_to_pcl2', output='screen',
        parameters=[{'input_topic': '/livox/lidar',
                     'output_topic': '/livox/lidar/pointcloud',
                     'frame_id': 'livox_frame'}])

    # 4. 静态外参 TF: base_link ← livox_frame (⚠ 占位, 标定后回填 fusion_params.yaml)
    static_tf_livox = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_base_to_livox',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.25',      # ⚠ 占位
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'])

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
                    os.path.join(share_dir, 'config', 'debug_flags.yaml')])

    perception_rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(share_dir, 'config', 'lidar_perception.rviz')],
        condition=IfCondition(rviz))

    # 6. TODO(§7): 通信节点 — 实现后在此启动; 降级模式下天空方块颜色输出 UNKNOWN

    return LaunchDescription([
        rviz_arg,
        lio_arg,
        LogInfo(msg="[br_perception] 降级模式启动 — §10.2 (仅雷达; 相机故障)"),
        static_tf_livox,
        livox_driver,
        point_lio,
        custom_to_pcl2,
        lidar_pipeline,
        perception_rviz_node,
    ])
