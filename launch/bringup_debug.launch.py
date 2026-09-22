#!/usr/bin/env python3
"""
相机层端到端联调 — ROBOCON 2027 BR 感知系统 (任务书 §10.4, P0-6)

**第一次端到端联调只能靠它** —— 在此之前相机层 6 个节点从来没有一起跑过。

启动内容（全部相机层节点）:
  1. image_preprocessor      双相机预处理 + letterbox
  2. mustika_detector        穆斯蒂卡（Mikasa V300W 排球）
  3. pillar_detector         深绿穆斯蒂卡柱 + 棕色核心支柱
  4. building_spot_detector  建筑位绿色方格
  5. camera_depth            D435i 深度辅助（独立回调，不阻塞主链）
  6. camera_perception       聚合薄壳 → /perception/camera/frame   ← P0-2 新增

⚠️ 本 launch **不起工业相机驱动** —— 因为仓库里还没有：
   /camera_wide/image_raw 与 /camera_tele/image_raw 这两路的生产者目前是 **Gazebo**
   （scripts/make_gazebo_world.py:180）；实机驱动还没接进来
   （模块进度.md 的 P2-14「卡接口确认: GigE / USB3 / 厂商 SDK?」）。
   → 实机联调时，那两路话题得**另外起**（厂商 SDK，或 ros2 bag play）。
     本 launch 只负责相机层节点本身，不假装知道工业相机怎么起。

D435i 是**可选**的（默认不起）—— 它在本层只承担**深度**一路。

前置:
  source /opt/ros/humble/setup.bash
  source ~/ws_livox/install/setup.bash          # 相机层用不到, 但整个包的 CMake 需要它
  source ~/br_perception/install/setup.bash

用法:
  # 主要场景: 相机层 + 另起的图像源（bag 回放 / 仿真 / 厂商 SDK）
  ros2 launch br_perception bringup_debug.launch.py

  # 连 D435i 一起起
  ros2 launch br_perception bringup_debug.launch.py realsense:=true

  # 进程级隔离（联调期定位崩溃/卡死时用, 代价是图像走序列化）
  ros2 launch br_perception bringup_debug.launch.py intra_process:=false

  # 看结果
  ros2 topic hz   /perception/camera/frame
  ros2 topic echo /perception/camera/frame --once

⚠️ 首次联调**必看的三件事**（否则很容易白折腾半天）:
   ① 深度槽永远是 IDLE —— camera_depth 的设计是"没收到 ROI 就不测量"，
      而 ROI 话题 /camera/depth/target_roi **在本 launch 里没有任何发布者**。
      所以首次联调看到深度无测量值是**正常的**，不是节点坏了。要测 MEASURE 通路:
        ros2 topic pub --once /camera/depth/target_roi sensor_msgs/msg/RegionOfInterest \
          "{x_offset: 600, y_offset: 300, width: 80, height: 80, do_rectify: false}"
   ② 六路检测话题任何一路没人发，聚合帧**照发**（这是设计 —— 见口径③），
      但对应槽的 slot_status[].state 会是 NEVER。**看槽状态，别看帧有没有到**。
   ③ RViz 里若什么都看不见，先确认用的是本文件的 config/camera_debug.rviz ——
      相机话题是 BEST_EFFORT，而 RViz 默认 Reliable，**不兼容就完全不显示**。

⚠️ 本文件**没有在本机验证过**（本机无 ROS）—— 按 doc/待编译验证清单_20260920.md 核对。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _resolve_launch(share_dir, launch_file):
    """在 launch/ 与 launch_ROS2/ 下找 launch 文件（livox 用后者，保持同款容错）"""
    for sub in ('launch', 'launch_ROS2', ''):
        candidate = os.path.join(share_dir, sub, launch_file) if sub \
            else os.path.join(share_dir, launch_file)
        if os.path.exists(candidate):
            return candidate
    raise RuntimeError(f"在 {share_dir} 下找不到 {launch_file}")


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


# ═══════════════════════════════════════════════════════════════════════════════
# 相机层的 6 个节点 —— **一处定义，两种跑法**（容器内 / 各自独立进程）
# ═══════════════════════════════════════════════════════════════════════════════
# 每项 = (节点名, plugin 全名, 可执行名, 是否用颜色阈值)
# ⚠️ 节点名必须与 config/camera_params.yaml 的参数块名**逐字一致** ——
#    否则参数匹配不上，节点静默用代码默认值（2026-09-20 踩过的坑）。
NODE_SPECS = [
    # name,                     plugin,                                            exe,                      uses_color
    ('image_preprocessor',      'br_perception::camera::ImagePreprocessor',        'image_preprocessor',     False),
    ('mustika_detector',        'br_perception::camera::MustikaDetector',          'mustika_detector',       True),
    ('pillar_detector',         'br_perception::camera::PillarDetector',           'pillar_detector',        True),
    ('building_spot_detector',  'br_perception::camera::BuildingSpotDetector',     'building_spot_detector', True),
    ('camera_depth',            'br_perception::camera::CameraDepthNode',          'camera_depth',           False),
    ('camera_perception',       'br_perception::camera::CameraPerceptionNode',     'camera_perception_node', False),
]


def generate_launch_description():
    pkg_share = get_package_share_directory('br_perception')
    camera_params = os.path.join(pkg_share, 'config', 'camera_params.yaml')

    # ⚠️⚠️ 颜色阈值文件必须给**绝对路径**。config/camera_params.yaml 里写的是
    #     相对路径 "config/color_thresholds.yaml"，而 color_utils.cpp:89 是
    #     `std::ifstream(yaml_path)` —— **相对启动时的 $PWD**，且节点里没有任何
    #     回退解析。launch 可以从任意目录起 → 检测器会找不到颜色文件
    #     （load_color_thresholds 抛异常）。所以这里按 install 空间解析成绝对路径覆盖。
    #     ⚠️ **只有 3 个节点用颜色**（mustika / pillar / building_spot）——
    #        见 NODE_SPECS 的 uses_color 列。给另外 3 个传是无用的未声明覆盖。
    color_yaml = os.path.join(pkg_share, 'config', 'color_thresholds.yaml')

    # ── launch 参数 ──
    intra_arg = DeclareLaunchArgument(
        'intra_process', default_value='true', choices=['true', 'false'],
        description='true = 6 个节点装进一个容器（图像零拷贝）; '
                    'false = 拆成 6 个独立进程（真隔离，代价是图像走序列化）')
    rs_arg = DeclareLaunchArgument(
        'realsense', default_value='false', choices=['true', 'false'],
        description='启动 D435i 驱动。默认 false —— 首次联调通常没有 D435i，'
                    '默认开着会刷一屏驱动错误日志')
    perception_rviz_arg = DeclareLaunchArgument(
        'perception_rviz', default_value='true', choices=['true', 'false'],
        description='启动 RViz2')
    # ⚠️ 这个 launch 参数与本文件里的**节点参数**同名，是有意的 ——
    #    这里不会冲突，因为**没有任何 include 会转发它**（本文件的 include 只有
    #    realsense 那一条，且只显式透传了本参数，不转发整个 launch context）。
    #    ⚠️ bringup_full.launch.py 踩过同类坑（rviz 参数被 include 污染），
    #       将来若在本文件里加 include，**先确认它不会把 use_sim_time 吃掉**。
    sim_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', choices=['true', 'false'],
        description='用仿真时钟（接 gazebo/bag 回放时设 true）')

    intra = LaunchConfiguration('intra_process')
    use_sim = LaunchConfiguration('use_sim_time')

    def make_params(uses_color):
        """一个节点的参数列表。⚠️ 顺序有意义: 后面的**覆盖**前面的。"""
        ps = [camera_params,
              # ⚠️ use_sim_time 不是节点自己的参数，但每个节点都得有一份，
              #    否则仿真下各节点时间戳不同源 —— 聚合薄壳的 stale_sec 全靠
              #    header.stamp 相减，不同源会算出天文数字甚至抛异常。
              {'use_sim_time': use_sim}]
        if uses_color:
            ps.append({'color_yaml_path': color_yaml})
        return ps

    # ── 跑法一: 单容器（默认）──
    # ⚠️ 相机层节点之间传的是**图像**（640×384×3 ≈ 700KB/帧 × 30Hz）。
    #    容器内启用 intra-process 后这些按指针传递、不再序列化 ——
    #    这正是"薄壳聚合比单体多一跳"那个代价的**具体消除手段**。
    #
    # ⚠️ 启用 intra-process 的**生命周期后果**（写在这里，将来换 loaned message 时要用）:
    #    消息按指针传递 → 下游持有 SharedPtr 就会**延长上游图像的生命周期**。
    #    聚合薄壳的 SlotData 正是长期持有（存到下一帧来为止）。
    #    将来若改用 **loaned message**（零拷贝的更进一步），**跨回调持有是禁止的** ——
    #    那时聚合薄壳必须改成"组帧时拷贝"，不能存指针。
    container = ComposableNodeContainer(
        name='camera_layer_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',   # mt = 多线程执行器
        # ⚠️ 用多线程的理由: **节点之间**并行 —— 三个检测器各跑各的 OpenCV，
        #    深度走独立回调（camera_depth_node.hpp 的硬约束是"独立回调，
        #    不阻塞主感知链"）。单线程会让它们互相排队，直接违反那条约束。
        #    ⚠️ **但多线程 ≠ 节点内并行**: 同一节点里 wide/tele 两个回调默认同属
        #       一个 MutuallyExclusive callback group，**仍然串行**。
        #       若将来发现 image_preprocessor 的宽/长焦成了瓶颈，得在**节点内部**
        #       加 callback group —— 换个容器执行器解决不了。
        #    ⚠️ 检测器内部有 state_mutex_ / diag_mutex_，聚合薄壳有 slot_mutex_，
        #       多线程下安全; **将来新增节点前先确认它也有锁**。
        composable_node_descriptions=[
            ComposableNode(
                package='br_perception', plugin=plugin, name=name,
                parameters=make_params(uses_color),
                extra_arguments=[{'use_intra_process_comms': intra}])
            for name, plugin, _exe, uses_color in NODE_SPECS
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim}],
    )

    # ── 跑法二: 6 个独立进程 ──
    # ⚠️ 这才是**真正的进程级隔离**：一个节点段错误只死自己，不带走整层。
    #    （容器跑法下 6 个节点同进程，一个崩全崩 —— 这是取舍，不是 bug。）
    #    代价: 节点之间的图像走 DDS 序列化/反序列化。
    standalone_nodes = [
        Node(package='br_perception', executable=exe, name=name,
             output='screen', parameters=make_params(uses_color))
        for name, _plugin, exe, uses_color in NODE_SPECS
    ]

    # ⚠️ 二选一在执行期完成（OpaqueFunction），不是在生成期 ——
    #    intra_process 是 LaunchConfiguration，生成期还没求值。
    #    ⚠️ 两种跑法**节点、参数、名字完全相同**，只有部署拓扑不同：
    #       这样在"隔离模式"里复现的问题，容器模式下也一定复现（反之亦然）。
    def _pick_topology(context):
        if context.launch_configurations.get('intra_process', 'true').lower() \
                in ('false', '0', 'no'):
            return standalone_nodes
        return [container]

    topology = OpaqueFunction(function=_pick_topology)

    # ⚠️ 用 OpaqueFunction 延迟解析: _include 内部要查包，而它在
    #    generate_launch_description() 阶段就执行 —— 早于 Condition 求值。
    #    直接写 condition=IfCondition(...) 的话，没装 realsense 时
    #    即使 realsense:=false 也会在生成阶段抛异常，条件形同虚设。
    def _realsense_if_enabled(context):
        if context.launch_configurations.get('realsense', 'false').lower() in ('false', '0', 'no'):
            return []
        # ⚠️ use_sim_time **必须一起透传**给驱动 —— 否则驱动的深度图时间戳用系统时钟，
        #    与本层其它节点不同源；聚合薄壳的 stale_sec 全靠 header.stamp 相减，
        #    轻则算出天文数字，重则 rclcpp::Time 混用不同 clock_type **直接抛异常**。
        #    （实机跑时它本来就是 false，这里透传是为了仿真/回放场景不踩坑。）
        return [_include(
            'realsense2_camera', 'rs_launch.py',
            launch_arguments={
                'camera_name': 'camera',        # → /camera/depth/image_rect_raw
                'enable_color': 'false',        # ⚠️ 本层用不到 D435i 的彩色
                'enable_depth': 'true',
                # ⚠️ 带点号的参数名在不同 realsense2_camera 版本间有过变化 ——
                #    已记入 doc/待编译验证清单_20260920.md，起不来先核对这个键名。
                'align_depth.enable': 'true',
                'use_sim_time': context.launch_configurations.get('use_sim_time', 'false'),
            },
            source_hint='D435i 驱动: sudo apt install ros-humble-realsense2-camera')]

    realsense = OpaqueFunction(function=_realsense_if_enabled)

    # ⚠️ 相机话题是 BEST_EFFORT，RViz 默认 Reliable **不兼容 = 什么都不显示** ——
    #    所以必须用 config/camera_debug.rviz（那份的显示项全部 Best Effort），
    #    **不能**复用 lidar_perception.rviz（那个连 Reliability Policy 都没写 = Reliable）。
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'camera_debug.rviz')],
        parameters=[{'use_sim_time': use_sim}],
        condition=IfCondition(LaunchConfiguration('perception_rviz')))

    return LaunchDescription([
        intra_arg,
        rs_arg,
        perception_rviz_arg,
        sim_arg,
        LogInfo(msg="[br_perception] 相机层联调启动 — §10.4 (P0-6)"),
        LogInfo(msg="  期望话题: /perception/camera/frame  (聚合薄壳输出)"),
        LogInfo(msg="  ⚠️ 深度槽默认恒为 IDLE —— ROI 话题 /camera/depth/target_roi 无发布者，"
                    "这是正常的；要测 MEASURE 见本文件 docstring 的 ros2 topic pub 示例"),
        topology,
        realsense,
        rviz_node,
    ])
