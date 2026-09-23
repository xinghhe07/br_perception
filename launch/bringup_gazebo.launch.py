#!/usr/bin/env python3
"""
Gazebo 仿真启动 — fishbot 载具 + Point-LIO (任务书 §4.1 / §十.2 的仿真验证通路)

链路:
  Gazebo (br_field.world + fishbot)
    ├─ /livox/lidar/pointcloud (PointCloud2, 16线 10Hz) → pointcloud2_to_livox_custom
    │                                                       └→ /livox/lidar (CustomMsg) → Point-LIO
    ├─ /imu/data (100Hz)
    ├─ /camera_wide/image_raw, /camera_tele/image_raw (30Hz)
    └─ /odom ← 车的差速插件
  /cmd_vel → 车的差速插件驱动巡游 (scripts/sim_patrol.py)

前置: source /opt/ros/humble/setup.bash && source ~/lio_ws/install/setup.bash
      && source ~/br_perception/install/setup.bash

用法:
  ros2 launch br_perception bringup_gazebo.launch.py                 # GUI + 自动巡游
  ros2 launch br_perception bringup_gazebo.launch.py gui:=false motion:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('br_perception')
    gz_dir = os.path.join(share, 'gazebo')
    world = os.path.join(gz_dir, 'worlds', 'br_field.world')
    models = os.path.join(gz_dir, 'models')
    pointlio_sim_cfg = os.path.join(share, 'config', 'pointlio_gazebo.yaml')
    patrol = os.path.join(share, 'scripts', 'sim_patrol.py')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true', choices=['true', 'false'],
        description='同时启动 gzclient (图形界面)')
    motion_arg = DeclareLaunchArgument(
        'motion', default_value='true', choices=['true', 'false'],
        description='自动巡游 (发布 /cmd_vel); false 时手动遥控/静止')
    rviz_arg = DeclareLaunchArgument(
        'perception_rviz', default_value='true', choices=['true', 'false'],
        description='启动 RViz2 (看 Point-LIO 的 /cloud_registered + /path)')
    gui = LaunchConfiguration('gui')
    motion = LaunchConfiguration('motion')
    rviz = LaunchConfiguration('perception_rviz')

    # Gazebo 模型搜索路径 (含 br_field / sensor_rig)
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', f"{models}:/usr/share/gazebo-11/models")

    # gzserver: 无图形也能跑雷达/IMU (相机渲染需要 GL, VM 上可能慢)
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world], output='screen')

    gzclient = ExecuteProcess(
        cmd=['gzclient'], output='screen', condition=IfCondition(gui))

    # TF: base_link ← 各传感器 (odom←base_link 由车的差速插件发)。
    # Gazebo 不给固定关节发 TF, 这几段必须由 launch 广播, 否则 rviz 报
    # "Fixed Frame [odom] ... livox_frame does not exist", fusion 的
    # coordinate_transformer 也查不到外参。
    # ⚠ 数值来自 models/fishbot/model.sdf 里各 <sensor> 的 <pose>, 改一边要同步另一边。
    #   (config/fusion_params.yaml §static_tf 是给实车的占位值, 实车到位后以那边为准)
    SENSOR_TF = [
        ('livox_frame',        0.0,   0.0, 0.2419, 0.0, 0.0, 0.0),
        ('imu_link',           0.0,   0.0, 0.1119, 0.0, 0.0, 0.0),
        # ⚠️ 2026-09-23: tele 原为 `-0.1 … yaw 3.14159`（车尾、朝后），随
        #    `models/fishbot/model.sdf` 一起改成"**都朝前**"（实机是一宽一长焦、都朝 +x）。
        #    两处**必须一致** —— 本表就是照 SDF 的 <pose> 抄的，改一边不改另一边，
        #    TF 查出来的相机位姿就与图像实际朝向不符，而 transform_to_world 会照错不误。
        ('camera_wide_frame', 0.1,   0.0, 0.1669, 0.0, 0.0, 0.0),
        ('camera_tele_frame', 0.1,   0.0, 0.1669, 0.0, 0.0, 0.0),
        ('camera_depth_frame', 0.105, 0.0, 0.125,  0.0, 0.0, 0.0),
    ]
    static_tfs = [
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'static_tf_base_to_{frame}',
            arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                       '--roll', str(r), '--pitch', str(p), '--yaw', str(y_) or '0.0',
                       '--frame-id', 'base_link', '--child-frame-id', frame])
        for frame, x, y, z, r, p, y_ in SENSOR_TF
    ]

    # PointCloud2 → CustomMsg 桥接 (仿真雷达 → Point-LIO 的 Livox 通路)
    bridge = Node(
        package='br_perception', executable='pointcloud2_to_livox_custom',
        name='pointcloud2_to_livox_custom', output='screen',
        parameters=[{'input_topic': '/livox/lidar/pointcloud',
                     'output_topic': '/livox/lidar',
                     'scan_lines': 16,
                     'frame_rate_hz': 10.0,
                     'frame_id': 'livox_frame',
                     # 仿真雷达垂直 FOV = ±0.26 rad (±14.9°), 与实机 Mid-360
                     # (-7°~+52°) 不同 → 必须显式传, 否则点会挤在少数几条 line 上
                     # (models/fishbot/model.sdf 里 livox 传感器的 vertical 段)
                     'elev_min_deg': -14.9,
                     'elev_max_deg': 14.9}])

    # Point-LIO (仿真配置; 参数覆盖与 lio_ws 的 mapping_mid360.launch.py 保持一致)
    point_lio = Node(
        package='point_lio', executable='pointlio_mapping', name='laserMapping',
        output='screen',
        parameters=[pointlio_sim_cfg, {
            'use_imu_as_input': False,
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 3,
            'space_down_sample': True,
            'filter_size_surf': 0.5,
            'filter_size_map': 0.5,
            'cube_side_length': 1000.0,
            'runtime_pos_log_enable': False,
        }])

    # 巡游轨迹 (闭环方框, 用于漂移评估)
    patrol_proc = ExecuteProcess(
        cmd=['python3', patrol, '--side', '2.0', '--laps', '2'],
        output='screen', condition=IfCondition(motion))

    # 仿真专用 rviz 配置: 显示项全部 Best Effort (Gazebo 传感器 QoS), 否则 rviz 报
    # "Global Status: Error" (QoS 不兼容, 收不到数据)
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(share, 'config', 'gazebo_sim.rviz')],
        condition=IfCondition(rviz))

    return LaunchDescription([
        gui_arg, motion_arg, rviz_arg,
        LogInfo(msg="[br_perception] Gazebo 仿真启动 — fishbot 载具 + Point-LIO (CustomMsg 桥接)"),
        set_model_path,
        *static_tfs,
        gzserver,
        gzclient,
        bridge,
        point_lio,
        patrol_proc,
        rviz_node,
    ])
