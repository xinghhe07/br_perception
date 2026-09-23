#!/usr/bin/env python3
"""
比赛模式全量启动 — ROBOCON 2027 BR 感知系统 (任务_v1.7 §10.1)

启动内容:
  1. Livox Mid-360 雷达驱动          (CustomMsg — Point-LIO 要求)
  2. Point-LIO 里程计节点            (ROS2 移植版 — §十.2 定案: 仅作纯里程计)
  3. Livox CustomMsg → PointCloud2 桥接 (感知管线要 PointCloud2)
  4. 静态外参 TF (base_link ← 传感器)  ⚠ 占位值, 标定后回填 fusion_params.yaml
  5. 雷达感知管线                    (br_perception: lidar_perception_node)
  6. 感知编排与降级                   (br_perception: perception_pipeline, 模块 27)
                                      → 组装 /perception/output (通信层唯一上行入口)
  7. 融合/通信/健康监控               ⬜ 待实现 (§4.5 fusion_node / §7 / §13)

前置 (必须先 source, 否则找不到驱动与里程计包):
  source /opt/ros/humble/setup.bash
  source ~/ws_livox/install/setup.bash        # livox_ros_driver2
  source ~/lio_ws/install/setup.bash          # point_lio
  source ~/br_perception/install/setup.bash

用法:
  ros2 launch br_perception bringup_full.launch.py
  ros2 launch br_perception bringup_full.launch.py perception_rviz:=false lio:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
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
    kwargs = {}
    if condition is not None:
        kwargs['condition'] = condition
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_resolve_launch(share, launch_file)),
        launch_arguments=(launch_arguments or {}).items(),
        **kwargs,
    )


def generate_launch_description():
    pkg_share = get_package_share_directory('br_perception')

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

    # ── 1. Livox Mid-360 雷达驱动 (CustomMsg — Point-LIO 的 lidar_type:1 要求) ──
    livox_driver = _include(
        'livox_ros_driver2', 'msg_MID360_launch.py',
        source_hint='雷达驱动在 ~/ws_livox, 见 ~/livox_lab_up.sh')

    # ── 2. Point-LIO 里程计节点 (§10.1 / §十.2: 降级为纯里程计, 绝对位姿靠场地先验) ──
    #    复用 lio_ws 的 launch 作为单一事实来源; 此处关掉其自带 rviz
    #    输出话题: /aft_mapped_to_init, 由 coordinate_transformer (§4.1) 消费
    # ⚠ 必须用 OpaqueFunction 延迟到"启动执行期"再解析: _include() 内部要调
    #   get_package_share_directory('point_lio'), 而它在 generate_launch_description()
    #   阶段就执行 —— 早于任何 condition 求值。若写成 _include(..., condition=IfCondition(lio)),
    #   没装 point_lio 时即使 lio:=false 也会在生成阶段抛 RuntimeError, 条件形同虚设。
    def _point_lio_if_enabled(context):
        if context.launch_configurations.get('lio', 'true').lower() in ('false', '0', 'no'):
            return []
        return [_include(
            'point_lio', 'mapping_mid360.launch.py',
            launch_arguments={'rviz': 'false'},
            source_hint='Point-LIO 在 ~/lio_ws (9000_point_lio_ros2_Mid-360)')]

    point_lio = OpaqueFunction(function=_point_lio_if_enabled)

    # ── 3. CustomMsg → PointCloud2 桥接 (感知管线订阅 /livox/lidar/pointcloud) ──
    custom_to_pcl2 = Node(
        package='br_perception', executable='livox_custom_to_pcl2',
        name='livox_custom_to_pcl2', output='screen',
        parameters=[{'input_topic': '/livox/lidar',
                     'output_topic': '/livox/lidar/pointcloud',
                     'frame_id': 'livox_frame'}])

    # ── 4. 静态外参 TF (⚠ 占位值 — 实测标定后同步改 config/fusion_params.yaml) ──
    #    TF 树 (§4.1): world ← odom ← base_link ← {livox_frame, camera_*}
    #    world←odom 与 odom←base_link 由 coordinate_transformer 广播
    static_tf_livox = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_base_to_livox',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.25',      # ⚠ 占位
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'])

    static_tf_cam_wide = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_base_to_camera_wide',
        arguments=['--x', '0.15', '--y', '0.0', '--z', '0.35',     # ⚠ 占位
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_wide_frame'])

    static_tf_cam_tele = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_base_to_camera_tele',
        arguments=['--x', '-0.15', '--y', '0.0', '--z', '0.35',    # ⚠ 占位
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '3.14159',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_tele_frame'])

    # ── 5. 雷达感知管线 (现有实现: 预处理→地面→圆柱→聚类→建筑位) ──
    #    直接起主节点, 不复用 lidar_perception.launch.py — 那个 launch 会发
    #    map→livox_frame 静态 TF, 与本 bringup 的 base_link 树冲突 (同帧双父)
    lidar_pipeline = Node(
        package='br_perception', executable='lidar_perception_node',
        name='lidar_perception_node', output='screen',
        # ⚠️ `debug_flags.yaml` **必须排在最后** —— ROS 2 参数**后者覆盖前者**，
        #    而它里面的 `debug_enabled` 要盖掉 `lidar_params.yaml:86` 的 `true`。
        #    比赛模式要求"调试全部关闭"（任务书 §1.7），那个 true 是给台架用的。
        #    ⚠️ 顺序写反 = 调试点云在比赛时照发，且**不会报任何错**。
        parameters=[os.path.join(pkg_share, 'config', 'lidar_params.yaml'),
                    os.path.join(pkg_share, 'config', 'field_geometry.yaml'),
                    os.path.join(pkg_share, 'config', 'debug_flags.yaml')])

    # ── 6. 感知编排与降级 (模块 27) ──
    #    ⚠️ **必须传两份 yaml**: fusion_params.yaml 提供 TF 的坐标系名与里程计参数
    #       (CoordinateTransformer 的 load_from_node 读它们)。缺了这份 → TF 取默认名
    #       → 与雷达结果的实际坐标系对不上 → **一帧都发不出去**(且不报错, 只报 TF 失败)。
    #    ⚠️ 本节点内部会起一个 CoordinateTransformer, 因此**它广播 odom→base_link /
    #       world→odom**。目前 fusion_node(模块21)不存在, 没人广播 → 这里正好补上;
    #       **fusion_node 落地后两个广播器会打架, 必须二选一**(见 pkg 的 perception_pipeline.hpp)。
    perception_pipeline = Node(
        package='br_perception', executable='perception_pipeline',
        name='perception_pipeline', output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'perception_params.yaml'),
                    os.path.join(pkg_share, 'config', 'fusion_params.yaml')])

    perception_rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'lidar_perception.rviz')],
        condition=IfCondition(rviz))

    # ── 6. 融合 / 通信 / 健康监控 ⬜ 待实现 ──
    #  §4.5 fusion_node          : 调用 coordinate_transformer → ... → PerceptionFrame
    #  §7   communication        : 串口协议编码 (§5.1), 目标点/期望路径下发
    #  §13  health_monitor        : 热管理 OnThermalThrottle / 看门狗
    # TODO(模块 21): fusion_node 实现后在此启动, 并加载 config/fusion_params.yaml:
    #   Node(package='br_perception', executable='fusion_node',
    #        parameters=[os.path.join(pkg_share, 'config', 'fusion_params.yaml')])
    # TODO(§13.3): 实时性 — 隔离核心 + 实时调度 (§10.1 任务清单最后两条):
    #   prefix='taskset -c 3 chrt -f 50'   # perception_pipeline 绑定隔离核

    return LaunchDescription([
        rviz_arg,
        lio_arg,
        LogInfo(msg="[br_perception] 比赛模式启动 — §10.1 (里程计: Point-LIO 纯里程计 + 场地先验)"),
        static_tf_livox,
        static_tf_cam_wide,
        static_tf_cam_tele,
        livox_driver,
        point_lio,
        custom_to_pcl2,
        lidar_pipeline,
        perception_pipeline,
        perception_rviz_node,
    ])
